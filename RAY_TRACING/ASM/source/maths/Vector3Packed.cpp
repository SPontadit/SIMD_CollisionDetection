#include "maths/Vector3Packed.h"

namespace maths
{

	Vector3Packed Vector3Packed::operator -(const Vector3Packed& rhs) const noexcept
	{
		return Vector3Packed(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	void Vector3Packed::normalize() noexcept
	{
		Vector4 x2 = x * x;
		Vector4 y2 = y * y;
		Vector4 z2 = z * z;

		Vector4 magnitudes = x2 + y2 + z2;
		magnitudes.sseVal = _mm_sqrt_ps(magnitudes.sseVal);

		x = x / magnitudes;
		y = y / magnitudes;
		z = z / magnitudes;
	}

	Vector4 dot(const Vector3Packed& lhs, const Vector3Packed& rhs) noexcept
	{
		Vector4 resX = lhs.x * rhs.x;
		Vector4 resY = lhs.y * rhs.y;
		Vector4 resZ = lhs.z * rhs.z;

		return resX + resY + resZ;
	}

} // namespace maths