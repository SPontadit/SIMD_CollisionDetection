#include "maths/Sphere.h"

#include <intrin.h>

namespace maths
{

	bool Sphere::hit(const Ray& ray) const noexcept
	{
		Vector3 oc = ray.origin - center;
		float b = dot(oc, ray.direction);
		float c = dot(oc, oc) - radius * radius;

		float discrimiant = b * b - c;

		return discrimiant >= 0.f;
	}

	int Sphere::hit(const RayPacked& rays) const noexcept
	{
		// Algorithm from https://www.iquilezles.org/blog/?p=2411
		// is done 4 times in parallel in SSE registers

		Vector3Packed center(center.x, center.y, center.z);
		Vector3Packed oc = rays.origins - center;
		Vector4 zeroes(0.f);

		Vector4 b = dot(oc, rays.directions);
		Vector4 c = dot(oc, oc) - Vector4(radius * radius);

		Vector4 discriminant = b * b - c;
		__m128 discriminantMask = _mm_cmpge_ps(discriminant.sseVal, zeroes.sseVal);
		// resultMask contains a bit set to 1 for each ray that hits the sphere
		unsigned int resultMask = _mm_movemask_ps(discriminantMask);

		// Return the number of bits set to 1 to get the number of rays that have hit
		return __popcnt(resultMask);
	}

} // namespace maths