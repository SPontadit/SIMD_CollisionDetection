#ifndef RAY_H_INCLUDED
#define RAY_H_INCLUDED

#include "maths/Vector3.h"
#include "maths/Vector3Packed.h"

namespace maths
{

	struct Ray
	{
		Ray(const Vector3& origin, const Vector3& direction)
			: origin(origin), direction(direction) { }

		Vector3 origin;
		Vector3 direction;

		//Vector3 at(float time);
	};

	struct alignas(16) RayPacked
	{
		RayPacked(const Vector3Packed& origins, const Vector3Packed& directions) noexcept
			: origins(origins), directions(directions) { }

		Vector3Packed origins;
		Vector3Packed directions;
	};

} // namespace maths

#endif // RAY_H_INCLUDED