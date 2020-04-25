#include "maths/Vector3.h"

#include "SSE.h"

namespace maths
{

    Vector3 Vector3::operator -() const noexcept
    {
        return Vector3(sse::negate(sseVal));
    }

    Vector3 Vector3::operator +(const Vector3& rhs) const noexcept
    {
        return Vector3(_mm_add_ps(sseVal, rhs.sseVal));
    }

    Vector3 Vector3::operator -(const Vector3& rhs) const noexcept
    {
        return Vector3(_mm_sub_ps(sseVal, rhs.sseVal));
    }

    Vector3 Vector3::operator *(const Vector3& rhs) const noexcept
    {
        return Vector3(_mm_mul_ps(sseVal, rhs.sseVal));
    }

    Vector3 Vector3::operator /(const Vector3& rhs) const noexcept
    {
        return Vector3(_mm_div_ps(sseVal, rhs.sseVal));
    }

    Vector3& Vector3::operator +=(const Vector3& rhs) noexcept
    {
        sseVal = _mm_add_ps(sseVal, rhs.sseVal);
        return *this;
    }

    Vector3& Vector3::operator -=(const Vector3& rhs) noexcept
    {
        sseVal = _mm_sub_ps(sseVal, rhs.sseVal);
        return *this;
    }

    Vector3& Vector3::operator *=(const Vector3& rhs) noexcept
    {
        sseVal = _mm_mul_ps(sseVal, rhs.sseVal);
        return *this;
    }

    Vector3& Vector3::operator /=(const Vector3& rhs) noexcept
    {
        sseVal = _mm_div_ps(sseVal, rhs.sseVal);
        return *this;
    }

    Vector3 Vector3::operator *(const float scalar) const noexcept
    {
        __m128 scalarVector = _mm_set_ps1(scalar);
        return Vector3(_mm_mul_ps(sseVal, scalarVector));
    }

    Vector3 Vector3::operator /(const float scalar) const noexcept
    {
        __m128 scalarVector = _mm_set_ps1(1.f / scalar);
        return Vector3(_mm_mul_ps(sseVal, scalarVector));
    }

    Vector3& Vector3::operator *=(const float scalar) noexcept
    {
        __m128 scalarVector = _mm_set_ps1(scalar);
        sseVal = _mm_mul_ps(sseVal, scalarVector);
        return *this;
    }

    Vector3& Vector3::operator /=(const float scalar) noexcept
    {
        __m128 scalarVector = _mm_set_ps1(1.f / scalar);
        sseVal = _mm_mul_ps(sseVal, scalarVector);
        return *this;
    }

    float Vector3::getSqrdMagnitude() const noexcept
    {
        return sse::getSqrdMagnitude(sseVal);
    }

    float Vector3::getMagnitude() const noexcept
    {
        return sse::getMagnitude(sseVal);
    }

    void Vector3::normalize() noexcept
    {
        /*__m128 magnitude = _mm_mul_ps(sseVal, sseVal);
        magnitude = _mm_hadd_ps(magnitude, magnitude);
        magnitude = _mm_hadd_ps(magnitude, magnitude);
        magnitude = _mm_sqrt_ps(magnitude);

        sseVal = _mm_div_ps(sseVal, magnitude);*/

        sseVal = sse::normalize(sseVal);
    }

    Vector3 Vector3::getNormalizedCopy() const noexcept
    {
        return Vector3(sse::normalize(sseVal));
    }

    float dot(const Vector3& lhs, const Vector3& rhs) noexcept
    {
        return sse::dot(lhs.sseVal, rhs.sseVal);
    }

    Vector3 cross(const Vector3& lhs, const Vector3& rhs) noexcept
    {
        // Create vector { y1, z1, x1 } from lhs
        __m128 lhsShuffled = _mm_shuffle_ps(lhs.sseVal, lhs.sseVal, 0b11001001);
        // Create vector { z2, x2, y2 } from rhs
        __m128 rhsShuffled = _mm_shuffle_ps(rhs.sseVal, rhs.sseVal, 0b11010010);
        // Create vector { y1*z2, z1*x2, x1*y2 } from shuffled vectors
        __m128 tmp0 = _mm_mul_ps(lhsShuffled, rhsShuffled);

        // Create vector { z1, x1, y1 } from lhsShuffled
        lhsShuffled = _mm_shuffle_ps(lhsShuffled, lhsShuffled, 0b11001001);
        // Create vector { y2, z2, x2 } from rhsShuffled
        rhsShuffled = _mm_shuffle_ps(rhsShuffled, rhsShuffled, 0b11010010);
        // Create vector { z1*y2, x1*z2, y1*x2 } from shuffled vectors
        __m128 tmp1 = _mm_mul_ps(lhsShuffled, rhsShuffled);

        return Vector3(_mm_sub_ps(tmp0, tmp1));
    }

    Vector3 lerp(const Vector3& from, const Vector3& to, float t) noexcept
    {
        return Vector3(sse::lerp(from.sseVal, to.sseVal, t));
    }

    std::ostream& operator <<(std::ostream & out, const Vector3& v) noexcept
    {
        return out << v.x << ' ' << v.y << ' ' << v.z;
    }

} // namespace maths