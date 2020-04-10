#ifndef _AABB_H_
#define _AABB_H_

#include <cfloat>
#include "Maths.h"
#include <xmmintrin.h>

struct AABB
{
    constexpr AABB()
        : minimum(FLT_MAX, FLT_MAX), maximum(FLT_MIN, FLT_MIN) { }

    constexpr AABB(const Vec2& minimum, const Vec2& maximum)
        : minimum(minimum), maximum(maximum) { }

    constexpr explicit AABB(const __m128 reg)
        : reg(reg) { }

    union {
        struct { Vec2 minimum, maximum; };
        __m128 reg;
    };
};

#endif