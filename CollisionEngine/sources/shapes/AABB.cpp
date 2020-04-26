#include "shapes/AABB.h"

#include <algorithm>

#include "GlobalVariables.h"
#include "render/Renderer.h"
#include "physics/PhysicEngine.h"

/**
        X      Y
m = x cos a  -sin a
    y sin a   cos a

aabb A -> local
aabb B -> result

B.min.x = min( m[X][x] * A.min.x, m[X][x] * A.max.x)
        + min( m[Y][x] * A.min.y, m[Y][x] * A.max.y)

B.min.y = min( m[X][y] * A.min.x, m[X][y] * A.max.x)
        + min( m[Y][y] * A.min.y, m[Y][y] * A.max.y)

B.max.x = max( m[X][x] * A.min.x, m[X][x] * A.max.x)
        + max( m[Y][x] * A.min.y, m[Y][x] * A.max.y)

B.max.y = max( m[X][y] * A.min.x, m[X][y] * A.max.x)
        + max( m[Y][y] * A.min.y, m[Y][y] * A.max.y)

**/

/**

float e0 = m[X][x] * A.min.x
float e1 = m[X][y] * A.min.x
float e2 = m[Y][x] * A.min.y
float e3 = m[Y][y] * A.min.y

                                        A.min.x / A.min.y / A.max.x / A.max.y


float f0 = m[X][x] * A.max.x
float f1 = m[X][y] * A.max.x
float f2 = m[Y][x] * A.max.y
float f3 = m[Y][y] * A.max.y

__m128 e, f;

__m128 result = _mm_min_ps(e, f);

if ( e0 < f0)
{
    B.min.x += e0;
    B.max.x += f0;
}
else
{
   B.min.x += f0;
   B.max.x += e0;
}

**/

/**

A a4 a3 a2 a1
B b4 b3 b2 b1

R = shuffle A B

R = B B A A

R = shuffle R B

R = B B B B

**/

float AABB::Surface() const noexcept
{
    return (-maximum.x - minimum.x) * (-maximum.y - minimum.y);
}

AABB AABB::Transform(__m128 position, __m128 rotation) const noexcept
{
    __m128 min = _mm_movelh_ps(reg, reg);
    __m128 max = _mm_movehl_ps(reg, reg);

    __m128 rot = _mm_shuffle_ps(rotation, rotation, _MM_SHUFFLE(3, 1, 2, 0));

    __m128 e = _mm_mul_ps(min, rot);
    __m128 f = _mm_mul_ps(max, rot);

    __m128 blendMask = _mm_cmpgt_ps(e, f);
    min = _mm_blendv_ps(e, f, blendMask);
    max = _mm_blendv_ps(f, e, blendMask);

    min = _mm_hadd_ps(min, min);
    max = _mm_hadd_ps(max, max);

    __m128 aabb = _mm_movelh_ps(min, max);

    AABB res = AABB(_mm_add_ps(aabb, position));
    res.maximum *= -1.0f;

    return res;
}

void AABB::DrawWorld(const AABB& A) noexcept
{
    Vec2 leftUp(A.minimum.x, -A.maximum.y);
    Vec2 leftDown(A.minimum);
    Vec2 rightUp(-A.maximum.x, -A.maximum.y);
    Vec2 rightDown(-A.maximum.x, A.minimum.y);

    gVars->pRenderer->DrawLine(leftUp, rightUp, 1.0f, 0.0f, 0.0f);
    gVars->pRenderer->DrawLine(rightUp, rightDown, 1.0f, 0.0f, 0.0f);
    gVars->pRenderer->DrawLine(rightDown, leftDown, 1.0f, 0.0f, 0.0f);
    gVars->pRenderer->DrawLine(leftDown, leftUp, 1.0f, 0.0f, 0.0f);
}

float AABB::GetSurface(const std::vector<AABB>& aabbs) noexcept
{
	AABB s = AABB::GetSurrounding(aabbs);

	Vec2 max{ -s.maximum.x, -s.maximum.y };
	Vec2 area(max - s.minimum);

	return area.x * area.y;
}

AABB AABB::GetSurrounding(const std::vector<AABB>& aabbs) noexcept
{
	__m128 surround = aabbs[0].reg;
	for (size_t i = 1; i < aabbs.size(); i++)
		surround = _mm_min_ps(surround, aabbs[i].reg);

	return AABB(surround);
}

int PackedAABB::Intersect(const PackedAABB& a, const PackedAABB& b) noexcept
{
    // This is 4 AABB-AABB overlap test done in parallel. We store -max values
    // for AABBs in world space so negate them here with signMask and a xor.
    const __m128 signMask = _mm_set_ps1(-0.f);

    // Each comparison evaluates to true if the AABBs are separated
    __m128 r0 = _mm_cmplt_ps(_mm_xor_ps(a.maximumX, signMask), b.minimumX);
    __m128 r1 = _mm_cmpgt_ps(a.minimumX, _mm_xor_ps(b.maximumX, signMask));
    __m128 r2 = _mm_cmplt_ps(_mm_xor_ps(a.maximumY, signMask), b.minimumY);
    __m128 r3 = _mm_cmpgt_ps(a.minimumY, _mm_xor_ps(b.maximumY, signMask));
    
    // Merge results in a single registers, one test evaluating to true is enough
    // to conclude the AABBs do not overlap so we combine them with a logical or
    int mask = _mm_movemask_ps(_mm_or_ps(_mm_or_ps(r0, r1), _mm_or_ps(r2, r3)));

    // A bit in mask is set to 1 if boxes do not overlap so flip it to have 1s 
    // where boxes do overlap and clear the remaining bits
    return ~mask & 0xF;
}

PackedAABB::PackedAABB(const AABB& toPack) noexcept
{
    minimumX = _mm_shuffle_ps(toPack.reg, toPack.reg, _MM_SHUFFLE(0, 0, 0, 0));
    minimumY = _mm_shuffle_ps(toPack.reg, toPack.reg, _MM_SHUFFLE(1, 1, 1, 1));
    maximumX = _mm_shuffle_ps(toPack.reg, toPack.reg, _MM_SHUFFLE(2, 2, 2, 2));
    maximumY = _mm_shuffle_ps(toPack.reg, toPack.reg, _MM_SHUFFLE(3, 3, 3, 3));
}

void Node4::SetAABB(size_t index, const AABB& aabb) noexcept
{
    packedAABBs.minimumX.m128_f32[index] = aabb.minimum.x;
    packedAABBs.minimumY.m128_f32[index] = aabb.minimum.y;
    packedAABBs.maximumX.m128_f32[index] = aabb.maximum.x;
    packedAABBs.maximumY.m128_f32[index] = aabb.maximum.y;
}

AABB Node4::GetAABB(size_t index) const noexcept
{
    return AABB({ packedAABBs.minimumX.m128_f32[index], packedAABBs.minimumY.m128_f32[index] },
                { packedAABBs.maximumX.m128_f32[index], packedAABBs.maximumY.m128_f32[index] });
}

AABB Leaf::GetSurroundingAABB(const Leaf* leaves, size_t leafCount) noexcept
{
    __m128 surround = leaves[0].aabb.reg;
    for (size_t i = 1; i < leafCount; i++)
        surround = _mm_min_ps(surround, leaves[i].aabb.reg);

    return AABB(surround);
}

int Leaf::SortCenterX(const void* a, const void* b) noexcept
{
    const Leaf* leafA = static_cast<const Leaf*>(a);
    const Leaf* leafB = static_cast<const Leaf*>(b);

    return leafA->aabb.minimum.x - leafA->aabb.maximum.x < leafB->aabb.minimum.x - leafB->aabb.maximum.x;
}

int Leaf::SortCenterY(const void* a, const void* b) noexcept
{
    const Leaf* leafA = static_cast<const Leaf*>(a);
    const Leaf* leafB = static_cast<const Leaf*>(b);

    return leafA->aabb.minimum.y - leafA->aabb.maximum.y < leafB->aabb.minimum.y - leafB->aabb.maximum.y;
}