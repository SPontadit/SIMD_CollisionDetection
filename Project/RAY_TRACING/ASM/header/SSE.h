#ifndef SSE_H_INCLUDED
#define SSE_H_INCLUDED

#include <immintrin.h>

namespace sse
{

    __m128 negate(const __m128 float4) noexcept;

    float getSqrdMagnitude(const __m128 float4) noexcept;
    float getMagnitude(const __m128 float4) noexcept;
    __m128 normalize(const __m128 float4) noexcept;

    float dot(__m128 lhs, __m128 rhs) noexcept;

    __m128 lerp(const __m128 from, const __m128 to, float t) noexcept;

} // namespace sse

#endif // SSE_H_INCLUDED