#ifndef VECTOR3_H_INCLUDED
#define VECTOR3_H_INCLUDED

#include <xmmintrin.h>
#include <ostream>

namespace maths
{

    struct alignas(16) Vector3
    {
        constexpr Vector3() noexcept : val{ 0.f, 0.f, 0.f } { }
        explicit constexpr Vector3(float f) noexcept : val{ f, f, f } { }
        constexpr Vector3(float x, float y, float z) noexcept : val{ x, y ,z } { }
        explicit constexpr Vector3(__m128 sseVal) noexcept : sseVal(sseVal) { }

        constexpr Vector3(const Vector3& other) noexcept = default;
        constexpr Vector3(Vector3&& other) noexcept = default;

        ~Vector3() noexcept = default;

        union
        {
            struct { float x, y, z; };
            struct { float r, g, b; };
            float val[3];
            __m128 sseVal;
        };

        Vector3& operator =(const Vector3& other) noexcept = default;
        Vector3& operator =(Vector3&& other) noexcept = default;

        Vector3 operator -() const noexcept;

        Vector3 operator +(const Vector3& rhs) const noexcept;
        Vector3 operator -(const Vector3& rhs) const noexcept;
        Vector3 operator *(const Vector3& rhs) const noexcept;
        Vector3 operator /(const Vector3& rhs) const noexcept;

        Vector3& operator +=(const Vector3& rhs) noexcept;
        Vector3& operator -=(const Vector3& rhs) noexcept;
        Vector3& operator *=(const Vector3& rhs) noexcept;
        Vector3& operator /=(const Vector3& rhs) noexcept;

        Vector3 operator *(const float scalar) const noexcept;
        Vector3 operator /(const float scalar) const noexcept;

        Vector3& operator *=(const float scalar) noexcept;
        Vector3& operator /=(const float scalar) noexcept;

        float getSqrdMagnitude() const noexcept;
        float getMagnitude() const noexcept;

        void normalize() noexcept;
        Vector3 getNormalizedCopy() const noexcept;
    };


    float dot(const Vector3& lhs, const Vector3& rhs) noexcept;
    Vector3 cross(const Vector3& lhs, const Vector3& rhs) noexcept;

    Vector3 lerp(const Vector3& from, const Vector3& to, float t) noexcept;

    std::ostream& operator <<(std::ostream& out, const Vector3& v) noexcept;

} // namespace maths

#endif // VECTOR3_H_INCLUDED