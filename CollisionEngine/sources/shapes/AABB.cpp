#include "shapes/AABB.h"

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





#include <immintrin.h>

AABB Transform(const AABB& A, Mat2 rotMat)
{
    __m128 min = _mm_set_ps(A.minimum.x, A.minimum.y, A.minimum.x, A.minimum.y);
    __m128 max = _mm_set_ps(A.maximum.x, A.maximum.y, A.maximum.x, A.maximum.y);

    __m128 rot = _mm_set_ps(rotMat.X.x, rotMat.Y.x, rotMat.X.y, rotMat.Y.y);

    __m128 e = _mm_mul_ps(min, rot);
    __m128 f = _mm_mul_ps(max, rot);

    __m128 mask = _mm_cmple_ps(e, f);
    unsigned int blendMask = _mm_movemask_ps(mask);
    min = _mm_blend_ps(e, f, blendMask);
    max = _mm_blend_ps(f, e, blendMask);

    min = _mm_hadd_ps(e, e);
    max = _mm_hadd_ps(f, f);

    return AABB(_mm_shuffle_ps(min, max, _MM_SHUFFLE(2, 0, 2, 0)));
}