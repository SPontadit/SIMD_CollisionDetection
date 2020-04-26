#ifndef VECTOR4_H_INCLUDED
#define VECTOR4_H_INCLUDED

#include <xmmintrin.h>
#include <ostream>

namespace maths
{

    struct alignas(16) Vector4
    {
        constexpr Vector4() noexcept : val{ 0.f, 0.f, 0.f, 0.f } { }
        explicit constexpr Vector4(float f) noexcept : val{ f, f, f, f } { }
        constexpr Vector4(float x, float y, float z, float w) noexcept : val{ x, y, z, w } { }
        explicit constexpr Vector4(__m128 sseVal) noexcept : sseVal(sseVal) { }

        constexpr Vector4(const Vector4& other) noexcept = default;
        constexpr Vector4(Vector4&& other) noexcept = default;

        ~Vector4() noexcept = default;

        union
        {
            struct { float x, y, z, w; };
            float val[4];
            __m128 sseVal;
        };

        Vector4& operator =(const Vector4& other) noexcept = default;
        Vector4& operator =(Vector4&& other) noexcept = default;

        Vector4 operator -() const noexcept;

        Vector4 operator +(const Vector4& rhs) const noexcept;
        Vector4 operator -(const Vector4& rhs) const noexcept;
        Vector4 operator *(const Vector4& rhs) const noexcept;
        Vector4 operator /(const Vector4& rhs) const noexcept;

        Vector4& operator +=(const Vector4& rhs) noexcept;
        Vector4& operator -=(const Vector4& rhs) noexcept;
        Vector4& operator *=(const Vector4& rhs) noexcept;
        Vector4& operator /=(const Vector4& rhs) noexcept;

        Vector4 operator *(const float scalar) const noexcept;
        Vector4 operator /(const float scalar) const noexcept;

        Vector4& operator *=(const float scalar) noexcept;
        Vector4& operator /=(const float scalar) noexcept;

        float getSqrdMagnitude() const noexcept;
        float getMagnitude() const noexcept;

        void normalize() noexcept;
        Vector4 getNormalizedCopy() const noexcept;
    };

    float dot(const Vector4& lhs, const Vector4& rhs) noexcept;

    Vector4 lerp(const Vector4& from, const Vector4& to, float t) noexcept;

    std::ostream& operator <<(std::ostream& out, const Vector4& v) noexcept;

} // namespace maths

#endif // VECTOR4_H_INCLUDED