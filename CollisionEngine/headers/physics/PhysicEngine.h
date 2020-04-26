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
	SPolygonPair(size_t _polyA, size_t _polyB) : polyA(_polyA), polyB(_polyB){}

	size_t	polyA;
	size_t	polyB;
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
	const AABB& GetWorldAABB(size_t index) const { return m_worldAABBs[index]; }
	const Node4* GetBVH4Nodes() const { return m_bvh4Nodes.data(); }

	bool useSAH = true;

private:
	friend class CPenetrationVelocitySolver;

	void						BuildAABBTree();
	int32_t						BVH2Recurse(Node2* nodes, int32_t& newNodeIndex, Leaf* xSortedLeaves, Leaf* ySortedLeaves, size_t leafCount);
	int32_t						BVH2ToBVH4(Node2* bvh2Nodes, int32_t currentNode2Index, Node4* bvh4Nodes, int32_t& newNode4Index);
	static void					DrawBVH2(const Node2* nodes, const size_t nodeCount);
	static void					DrawBVH4(const Node4* nodes, const size_t nodeCount);

	void						CollisionBroadPhase();
	void						CollisionNarrowPhase();

	bool						SIMD_Set_OBBCollisionTest(CPolygonPtr p1, CPolygonPtr p2) const noexcept;
	bool						SISD_OBBCollisionTest(CPolygonPtr p1, CPolygonPtr p2) const noexcept;
	bool						SIMD_Set_Shuffle_OBBCollisionTest(CPolygonPtr p1, CPolygonPtr p2) const noexcept;
	bool						SIMD_Shuffle_OBBCollisionTest(__m128 pos, __m128 extent, __m128 rotX, __m128 rotY) const noexcept;

	bool						m_active = true;

	// Collision detection
	IBroadPhase*				m_broadPhase;
	std::vector<SPolygonPair>	m_pairsToCheck;
	std::vector<SCollision>		m_collidingPairs;

	std::vector<AABB> m_localAABBs;
	std::vector<AABB> m_worldAABBs;
	std::vector<Leaf> m_xSortedLeaves;
	std::vector<Leaf> m_ySortedLeaves;
	std::vector<Node2> m_bvh2Nodes;
	std::vector<Node4> m_bvh4Nodes;
};

#endif