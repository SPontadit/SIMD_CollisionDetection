#include "shapes/AABB.h"

#include "GlobalVariables.h"
#include "render/Renderer.h"

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
    __m128 min = _mm_set_ps(minimum.x, minimum.y, minimum.x, minimum.y);
    __m128 max = _mm_set_ps(maximum.x, maximum.y, maximum.x, maximum.y);

    __m128 rot = _mm_set_ps(rotation.X.x, rotation.Y.x, rotation.X.y, rotation.Y.y);

    __m128 e = _mm_mul_ps(min, rot);
    __m128 f = _mm_mul_ps(max, rot);

    __m128 blendMask = _mm_cmple_ps(e, f);
    min = _mm_blendv_ps(e, f, blendMask);
    max = _mm_blendv_ps(f, e, blendMask);

    min = _mm_hadd_ps(e, e);
    max = _mm_hadd_ps(f, f);

    __m128 aabb = _mm_shuffle_ps(min, max, _MM_SHUFFLE(2, 0, 2, 0));

    __m128 pos = _mm_set_ps(position.x, position.y, position.x, position.y);

    return AABB(_mm_add_ps(aabb, pos));
}

void AABB::Draw(const AABB& A) noexcept
{
    Vec2 leftUp(A.minimum.x, A.maximum.y);
    Vec2 leftDown(A.minimum);
    Vec2 rightUp(A.maximum);
    Vec2 rightDown(A.maximum.x, A.minimum.y);

    gVars->pRenderer->DrawLine(leftUp, rightUp, 1.0f, 0.0f, 0.0f);
    gVars->pRenderer->DrawLine(rightUp, rightDown, 1.0f, 0.0f, 0.0f);
    gVars->pRenderer->DrawLine(rightDown, leftDown, 1.0f, 0.0f, 0.0f);
    gVars->pRenderer->DrawLine(leftDown, leftUp, 1.0f, 0.0f, 0.0f);
}