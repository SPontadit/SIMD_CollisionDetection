#ifndef SPHERE_H_INCLUDED
#define SPHERE_H_INCLUDED

#include "maths/Vector3.h"
#include "maths/Ray.h"

namespace maths
{

	struct alignas(16) Sphere
	{
		Sphere(const Vector3& center, float radius) noexcept
			: center(center), radius(radius) { }

		Vector3 center;
		float radius;

		bool hit(const Ray& ray) const noexcept;
		// Returns how many rays in the RayPacked struct have hit the Sphere
		int hit(const RayPacked& rays) const noexcept;
	};

} // namespace maths

#endif // SPHERE_H_INCLUDED