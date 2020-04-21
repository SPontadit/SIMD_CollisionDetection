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

AABB AABB::Transform(Vec2 position, Mat2 rotation) const noexcept
{
    __m128 min = _mm_set_ps(minimum.y, minimum.x, minimum.y, minimum.x);
    __m128 max = _mm_set_ps(maximum.y, maximum.x, maximum.y, maximum.x);

    __m128 rot = _mm_set_ps(rotation.Y.y, rotation.X.y, rotation.Y.x, rotation.X.x);

    __m128 e = _mm_mul_ps(min, rot);
    __m128 f = _mm_mul_ps(max, rot);

    __m128 blendMask = _mm_cmpgt_ps(e, f);
    min = _mm_blendv_ps(e, f, blendMask);
    max = _mm_blendv_ps(f, e, blendMask);

    min = _mm_hadd_ps(min, min);
    max = _mm_hadd_ps(max, max);

    __m128 aabb = _mm_shuffle_ps(min, max, _MM_SHUFFLE(1, 0, 1, 0));

    __m128 pos = _mm_set_ps(position.y, position.x, position.y, position.x);

    AABB res = AABB(_mm_add_ps(aabb, pos));
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

int AABB::Intersect(const PackedAABB test, const PackedAABB node) noexcept
{
    __m128 r0 = _mm_cmplt_ps(test.maximumX, node.minimumX);
    __m128 r1 = _mm_cmpgt_ps(test.minimumX, node.maximumX);
    __m128 r2 = _mm_cmplt_ps(test.maximumY, node.minimumY);
    __m128 r3 = _mm_cmpgt_ps(test.minimumY, node.maximumY);
    
    int mask = _mm_movemask_ps(_mm_or_ps(_mm_or_ps(_mm_or_ps(r0, r1), r2), r3));
    
    return ~mask & 0xF;
}