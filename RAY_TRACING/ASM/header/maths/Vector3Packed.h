#ifndef VECTOR3PACKED_H_INCLUDED
#define VECTOR3PACKED_H_INCLUDED

#include "maths/Vector4.h"

namespace maths
{

	struct alignas(16) Vector3Packed
	{
		constexpr Vector3Packed(const Vector4& x, const Vector4& y, const Vector4& z) noexcept
			: x(x), y(y), z(z) { }
		constexpr Vector3Packed(float x, float y, float z) noexcept
			: x(x), y(y), z(z) { } 

		Vector4 x;
		Vector4 y;
		Vector4 z;

		Vector3Packed operator -(const Vector3Packed& rhs) const noexcept;

		void normalize() noexcept;
	};

	Vector4 dot(const Vector3Packed&, const Vector3Packed& rhs) noexcept;

} // namespace maths

#endif // VECTOR3PACKED_H_INCLUDED