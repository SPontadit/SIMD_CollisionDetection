#ifndef _PHYSIC_ENGINE_H_
#define _PHYSIC_ENGINE_H_

#include <vector>
#include <unordered_map>
#include "Maths.h"
#include "shapes/Polygon.h"
#include "shapes/AABB.h"

class IBroadPhase;

struct SPolygonPair
{
	SPolygonPair(CPolygonPtr _polyA, CPolygonPtr _polyB) : polyA(_polyA), polyB(_polyB){}

	CPolygonPtr	polyA;
	CPolygonPtr	polyB;
};

struct SCollision
{
	SCollision() = default;
	SCollision(CPolygonPtr _polyA, CPolygonPtr _polyB, Vec2	_point, Vec2 _normal, float _distance)
		: polyA(_polyA), polyB(_polyB), point(_point), normal(_normal), distance(_distance){}

	CPolygonPtr	polyA, polyB;

	Vec2	point;
	Vec2	normal;
	float	distance;
};

class CPhysicEngine
{
public:
	void	Reset();
	void	Activate(bool active);

	void	DetectCollisions();

	void	Step(float deltaTime);

	template<typename TFunctor>
	void	ForEachCollision(TFunctor functor)
	{
		for (const SCollision& collision : m_collidingPairs)
		{
			functor(collision);
		}
	}

	void AddLocalAABB(const AABB& aabb);
	void RemoveLocalAABB(size_t index);

	bool useSAH = true;

private:
	friend class CPenetrationVelocitySolver;

	void						BuildAABBTree();
	void						BuildAABBTree_Internal(Node** tree, std::vector<AABB>& aabbs);
	Node4*						BVH2ToBVH4(Node* node2);
	void						CollisionBroadPhase();
	void						CollisionNarrowPhase();

	bool						SIMD_OBBCollisionTest(CPolygonPtr p1, CPolygonPtr p2) const noexcept;
	bool						SISD_OBBCollisionTest(CPolygonPtr p1, CPolygonPtr p2) const noexcept;

	bool						m_active = true;

	// Collision detection
	IBroadPhase*				m_broadPhase;
	std::vector<SPolygonPair>	m_pairsToCheck;
	std::vector<SCollision>		m_collidingPairs;

	std::vector<AABB> m_localAABBs;
	std::vector<AABB> m_worldAABBs;
};

#endif