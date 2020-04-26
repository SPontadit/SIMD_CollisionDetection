#include "maths/Vector4.h"

#include "SSE.h"

namespace maths
{

    Vector4 Vector4::operator -() const noexcept
    {
        return Vector4(sse::negate(sseVal));
    }

    Vector4 Vector4::operator +(const Vector4& rhs) const noexcept
    {
        return Vector4(_mm_add_ps(sseVal, rhs.sseVal));
    }

    Vector4 Vector4::operator -(const Vector4& rhs) const noexcept
    {
        return Vector4(_mm_sub_ps(sseVal, rhs.sseVal));
    }

    Vector4 Vector4::operator *(const Vector4& rhs) const noexcept
    {
        return Vector4(_mm_mul_ps(sseVal, rhs.sseVal));
    }

    Vector4 Vector4::operator /(const Vector4& rhs) const noexcept
    {
        return Vector4(_mm_div_ps(sseVal, rhs.sseVal));
    }

    Vector4& Vector4::operator +=(const Vector4& rhs) noexcept
    {
        sseVal = _mm_add_ps(sseVal, rhs.sseVal);
        return *this;
    }

    Vector4& Vector4::operator -=(const Vector4& rhs) noexcept
    {
        sseVal = _mm_sub_ps(sseVal, rhs.sseVal);
        return *this;
    }

    Vector4& Vector4::operator *=(const Vector4& rhs) noexcept
    {
        sseVal = _mm_mul_ps(sseVal, rhs.sseVal);
        return *this;
    }

    Vector4& Vector4::operator /=(const Vector4& rhs) noexcept
    {
        sseVal = _mm_div_ps(sseVal, rhs.sseVal);
        return *this;
    }

    Vector4 Vector4::operator *(const float scalar) const noexcept
    {
        __m128 scalarVector = _mm_set_ps1(scalar);
        return Vector4(_mm_mul_ps(sseVal, scalarVector));
    }

    Vector4 Vector4::operator /(const float scalar) const noexcept
    {
        __m128 scalarVector = _mm_set_ps1(1.f / scalar);
        return Vector4(_mm_mul_ps(sseVal, scalarVector));
    }

    Vector4& Vector4::operator *=(const float scalar)  noexcept
    {
        __m128 scalarVector = _mm_set_ps1(scalar);
        sseVal = _mm_mul_ps(sseVal, scalarVector);
        return *this;
    }

    Vector4& Vector4::operator /=(const float scalar)  noexcept
    {
        __m128 scalarVector = _mm_set_ps1(1.f / scalar);
        sseVal = _mm_mul_ps(sseVal, scalarVector);
        return *this;
    }

    float Vector4::getSqrdMagnitude() const noexcept
    {
        return sse::getSqrdMagnitude(sseVal);
    }

    float Vector4::getMagnitude() const noexcept
    {
        return sse::getMagnitude(sseVal);
    }

    void Vector4::normalize() noexcept
    {
        /*__m128 magnitude = _mm_mul_ps(sseVal, sseVal);
        magnitude = _mm_hadd_ps(magnitude, magnitude);
        magnitude = _mm_hadd_ps(magnitude, magnitude);
        magnitude = _mm_sqrt_ps(magnitude);

        sseVal = _mm_div_ps(sseVal, magnitude);*/

        sseVal = sse::normalize(sseVal);
    }

    Vector4 Vector4::getNormalizedCopy() const noexcept
    {
        return Vector4(sse::normalize(sseVal));
    }

    float dot(const Vector4& lhs, const Vector4& rhs) noexcept
    {
        return sse::dot(lhs.sseVal, rhs.sseVal);
    }

    Vector4 lerp(const Vector4& from, const Vector4& to, float t) noexcept
    {
        return Vector4(sse::lerp(from.sseVal, to.sseVal, t));
    }

    std::ostream& operator <<(std::ostream& out, const Vector4& v) noexcept
    {
        return out << v.x << ' ' << v.y << ' ' << v.z << ' ' << v.w;
    }

} // namespace maths