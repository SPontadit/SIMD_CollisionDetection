#include "SSE.h"

#include <immintrin.h>

namespace sse
{
    __m128 negate(const __m128 float4) noexcept
    {
        return _mm_xor_ps(float4, _mm_set_ps1(-0.f));
    }

    float getSqrdMagnitude(const __m128 float4) noexcept
    {
        __m128 res = _mm_mul_ps(float4, float4);
        res = _mm_hadd_ps(res, res);
        res = _mm_hadd_ps(res, res);

        return res.m128_f32[0];
    }

    float getMagnitude(const __m128 float4) noexcept
    {
        __m128 res = _mm_mul_ps(float4, float4);
        res = _mm_hadd_ps(res, res);
        res = _mm_hadd_ps(res, res);
        res = _mm_sqrt_ps(res);

        return res.m128_f32[0];
    }

    __m128 normalize(const __m128 float4) noexcept
    {
        __m128 magnitude = _mm_mul_ps(float4, float4);
        magnitude = _mm_hadd_ps(magnitude, magnitude);
        magnitude = _mm_hadd_ps(magnitude, magnitude);
        magnitude = _mm_sqrt_ps(magnitude);

        return _mm_div_ps(float4, magnitude);
    }

    float dot(__m128 lhs, __m128 rhs) noexcept
    {
        __m128 res = _mm_mul_ps(lhs, rhs);
        res = _mm_hadd_ps(res, res);
        res = _mm_hadd_ps(res, res);

        return res.m128_f32[0];
    }

    __m128 lerp(const __m128 from, const __m128 to, float t) noexcept
    {
        __m128 diff = _mm_sub_ps(to, from);
        __m128 factor = _mm_set_ps1(t);

        return _mm_fmadd_ps(diff, factor, from);
    }

} // namespace sse