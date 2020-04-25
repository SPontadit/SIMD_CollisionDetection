#include "physics/PhysicEngine.h"

#include <iostream>
#include <string>
#include <algorithm>
#include "GlobalVariables.h"
#include "World.h"
#include "render/Renderer.h" // for debugging only
#include "Timer.h"

#include "physics/BroadPhase.h"
#include "physics/BroadPhaseAABBTree.h"


void	CPhysicEngine::Reset()
{
	m_pairsToCheck.clear();
	m_collidingPairs.clear();

	m_localAABBs.clear();
	m_worldAABBs.clear();

	m_active = true;

	m_broadPhase = new CBroadPhaseAABBTree();
}

void	CPhysicEngine::Activate(bool active)
{
	m_active = active;
}

void	CPhysicEngine::DetectCollisions()
{
	CTimer timer;
	timer.Start();
	CollisionBroadPhase();
	timer.Stop();
	if (gVars->bDebug)
	{
		gVars->pRenderer->DisplayText("Collision broadphase duration " + std::to_string(timer.GetDuration() * 1000.0f) + " ms");
	}

	timer.Start();
	CollisionNarrowPhase();
	timer.Stop();
	if (gVars->bDebug)
	{
		gVars->pRenderer->DisplayText("Collision narrowphase duration " + std::to_string(timer.GetDuration() * 1000.0f) + " ms, collisions : " + std::to_string(m_collidingPairs.size()));
	}
}

void	CPhysicEngine::Step(float deltaTime)
{
	if (!m_active)
	{
		//return;
	}

	BuildAABBTree();

	DetectCollisions();
}

void	CPhysicEngine::AddLocalAABB(const AABB& aabb)
{
	m_localAABBs.push_back(aabb);
}

void CPhysicEngine::RemoveLocalAABB(size_t index)
{
	m_localAABBs[index] = m_localAABBs[m_localAABBs.size() - 1];
	m_localAABBs.pop_back();
}

void CPhysicEngine::DrawBVH2(const Node2* nodes, const size_t nodeCount)
{
	for (size_t i = 0; i < nodeCount; i++)
	{
		AABB::DrawWorld(nodes[i].childAABBs[0]);
		AABB::DrawWorld(nodes[i].childAABBs[1]);
	}
}

void CPhysicEngine::DrawBVH4(const Node4* nodes, const size_t nodeCount)
{
	for (size_t i = 0; i < nodeCount; i++)
	{
		if (nodes[i].children[0].index != -1) AABB::DrawWorld(nodes[i].GetAABB(0));
		if (nodes[i].children[1].index != -1) AABB::DrawWorld(nodes[i].GetAABB(1));
		if (nodes[i].children[2].index != -1) AABB::DrawWorld(nodes[i].GetAABB(2));
		if (nodes[i].children[3].index != -1) AABB::DrawWorld(nodes[i].GetAABB(3));
	}
}

void	CPhysicEngine::BuildAABBTree()
{
	const size_t objectCount = m_localAABBs.size();

	m_worldAABBs.resize(objectCount);
	m_xSortedLeaves.resize(objectCount);
	m_ySortedLeaves.resize(objectCount);
	
	CPolygon& poly = gVars->pWorld->GetPolygons();

	for (size_t i = 0; i < objectCount; i++)
	{
		size_t arrayIdx = floor(i / 4);
		size_t registerIdx = i % 4;

		__m128 pos;
		// X, X, Y, Y
		if (registerIdx == 0)
			pos = _mm_shuffle_ps(poly.positionX[arrayIdx], poly.positionY[arrayIdx], _MM_SHUFFLE(0, 0, 0, 0));
		else if (registerIdx == 1)
			pos = _mm_shuffle_ps(poly.positionX[arrayIdx], poly.positionY[arrayIdx], _MM_SHUFFLE(1, 1, 1, 1));
		else if (registerIdx == 2)
			pos = _mm_shuffle_ps(poly.positionX[arrayIdx], poly.positionY[arrayIdx], _MM_SHUFFLE(2, 2, 2, 2));
		else if (registerIdx == 3)
			pos = _mm_shuffle_ps(poly.positionX[arrayIdx], poly.positionY[arrayIdx], _MM_SHUFFLE(3, 3, 3, 3));
		// X, Y, X, Y
		pos = _mm_shuffle_ps(pos, pos, _MM_SHUFFLE(2, 0, 2, 0));

		AABB worldAABB = m_localAABBs[i].Transform(pos, poly.registerRotation[i]);

		m_worldAABBs[i] = worldAABB;
		m_ySortedLeaves[i] = m_xSortedLeaves[i] = Leaf(worldAABB, i);
	}

	size_t nodeCount = objectCount  - 1;
	m_bvh2Nodes.resize(nodeCount);
	m_bvh4Nodes.resize(nodeCount);

	// Build BVH2
	int32_t newNodeIndex = 0;
	BVH2Recurse(m_bvh2Nodes.data(), newNodeIndex, m_xSortedLeaves.data(), m_ySortedLeaves.data(), objectCount);

	//if (gVars->bDebug)
	//	DrawBVH2(bvh2Nodes, nodeCount);

	// Convert BVH2 to BVH4
	newNodeIndex = 0;
	BVH2ToBVH4(m_bvh2Nodes.data(), 0, m_bvh4Nodes.data(), newNodeIndex);

	if (gVars->bDebug)
		DrawBVH4(m_bvh4Nodes.data(), newNodeIndex);
}

int32_t CPhysicEngine::BVH2Recurse(Node2* nodes, int32_t& newNodeIndex, Leaf* xSortedLeaves, Leaf* ySortedLeaves, size_t leafCount)
{
	int32_t nodeIndex = newNodeIndex++;
	Node2* node = new(nodes + nodeIndex) Node2();

	std::qsort(xSortedLeaves, leafCount, sizeof(Leaf), Leaf::SortCenterX);
	std::qsort(ySortedLeaves, leafCount, sizeof(Leaf), Leaf::SortCenterY);

	const int frontCount = leafCount / 2;
	const int backCount = leafCount - frontCount;

	const AABB xFront = Leaf::GetSurroundingAABB(xSortedLeaves, frontCount);
	const AABB xBack = Leaf::GetSurroundingAABB(xSortedLeaves + frontCount, backCount);
	const AABB yFront = Leaf::GetSurroundingAABB(ySortedLeaves, frontCount);
	const AABB yBack = Leaf::GetSurroundingAABB(ySortedLeaves + frontCount, backCount);

	const float xSAH = xFront.Surface() + xBack.Surface();
	const float ySAH = yFront.Surface() + yBack.Surface();

	if (xSAH < ySAH)
	{
		memcpy(ySortedLeaves, xSortedLeaves, leafCount * sizeof(Leaf));

		node->childAABBs[0] = xFront;
		node->childAABBs[1] = xBack;

		if (frontCount > 1)
			node->children[0].index = BVH2Recurse(nodes, newNodeIndex, xSortedLeaves, ySortedLeaves, frontCount);
		else
		{
			node->children[0].index = xSortedLeaves->polyIndex;
			node->children[0].isLeaf = true;
		}

		if (backCount > 1)
			node->children[1].index = BVH2Recurse(nodes, newNodeIndex, xSortedLeaves + frontCount, ySortedLeaves + frontCount, backCount);
		else
		{
			node->children[1].index = xSortedLeaves[frontCount].polyIndex;
			node->children[1].isLeaf = true;
		}
	}
	else
	{
		memcpy(xSortedLeaves, ySortedLeaves, leafCount * sizeof(Leaf));

		node->childAABBs[0] = yFront;
		node->childAABBs[1] = yBack;

		if (frontCount > 1)
			node->children[0].index = BVH2Recurse(nodes, newNodeIndex, xSortedLeaves, ySortedLeaves, frontCount);
		else
		{
			node->children[0].index = ySortedLeaves->polyIndex;
			node->children[0].isLeaf = true;
		}

		if (backCount > 1)
			node->children[1].index = BVH2Recurse(nodes, newNodeIndex, xSortedLeaves + frontCount, ySortedLeaves + frontCount, backCount);
		else
		{
			node->children[1].index = ySortedLeaves[frontCount].polyIndex;
			node->children[1].isLeaf = true;
		}
	}

	return nodeIndex;
}

int32_t CPhysicEngine::BVH2ToBVH4(Node2* bvh2Nodes, int32_t currentNode2Index, Node4* bvh4Nodes, int32_t& newNode4Index)
{
	int32_t node4Index = newNode4Index++;
	Node4* node4 = new(bvh4Nodes + node4Index) Node4;

	Node2* node2 = bvh2Nodes + currentNode2Index;

	ChildID childrenTemp[4];
	float areas[4];

	childrenTemp[0] = node2->children[0];
	areas[0] = node2->childAABBs[0].Surface();
	node4->SetAABB(0, node2->childAABBs[0]);

	childrenTemp[1] = node2->children[1];
	areas[1] = node2->childAABBs[0].Surface();
	node4->SetAABB(1, node2->childAABBs[1]);

	size_t childCount = 2;

	while (childCount < 4)
	{
		int64_t bestIdx = -1;
		float bestArea = -std::numeric_limits<float>::infinity();

		for (size_t i = 0; i < childCount; i++)
		{
			if (childrenTemp[i].isLeaf)
				continue;

			float area = areas[i];
			if (area > bestArea)
			{
				bestArea = area;
				bestIdx = i;
			}
		}

		if (bestIdx < 0)
			break;

		Node2* bestNode = bvh2Nodes + childrenTemp[bestIdx].index;

		childrenTemp[bestIdx] = bestNode->children[0];
		areas[bestIdx] = bestNode->childAABBs[0].Surface();
		node4->SetAABB(bestIdx, bestNode->childAABBs[0]);

		childrenTemp[childCount] = bestNode->children[1];
		areas[childCount] = bestNode->childAABBs[1].Surface();
		node4->SetAABB(childCount, bestNode->childAABBs[1]);

		childCount++;
	}

	for (size_t i = 0; i < childCount; i++)
	{
		node4->children[i] = childrenTemp[i];
		if (!childrenTemp[i].isLeaf)
			node4->children[i].index = BVH2ToBVH4(bvh2Nodes, childrenTemp[i].index, bvh4Nodes, newNode4Index);
	}

	return node4Index;
}

void	CPhysicEngine::CollisionBroadPhase()
{
	m_pairsToCheck.clear();
	m_broadPhase->GetCollidingPairsToCheck(m_pairsToCheck);
}

bool CPhysicEngine::SIMD_Shuffle_OBBCollisionTest(__m128 pos, __m128 extent, __m128 rotX, __m128 rotY) const noexcept
{
	__m128 p1x = _mm_shuffle_ps(pos, pos, _MM_SHUFFLE(0, 0, 0, 0));
	__m128 p1y = _mm_shuffle_ps(pos, pos, _MM_SHUFFLE(1, 1, 1, 1));
	__m128 p2x = _mm_shuffle_ps(pos, pos, _MM_SHUFFLE(2, 2, 2, 2));
	__m128 p2y = _mm_shuffle_ps(pos, pos, _MM_SHUFFLE(3, 3, 3, 3));

	__m128 dx = _mm_sub_ps(p1x, p2x);
	__m128 dy = _mm_sub_ps(p1y, p2y);

	__m128 ex = _mm_shuffle_ps(extent, extent, _MM_SHUFFLE(0, 0, 2, 2));
	__m128 ey = _mm_shuffle_ps(extent, extent, _MM_SHUFFLE(1, 1, 3, 3));

	__m128 tmp_e = _mm_blend_ps(ex, ey, 0b1010);
	__m128 e = _mm_shuffle_ps(tmp_e, tmp_e, _MM_SHUFFLE(0, 1, 2, 3));
	
	__m128 rotXx = _mm_shuffle_ps(rotX, rotX, _MM_SHUFFLE(3, 3, 1, 1));
	__m128 rotXy = _mm_shuffle_ps(rotY, rotY, _MM_SHUFFLE(3, 3, 1, 1));
	__m128 rotYx = _mm_shuffle_ps(rotX, rotX, _MM_SHUFFLE(2, 2, 0, 0));
	__m128 rotYy = _mm_shuffle_ps(rotY, rotY, _MM_SHUFFLE(2, 2, 0, 0));

	__m128 rx = _mm_add_ps(_mm_mul_ps(ex, rotXx), _mm_mul_ps(ey, rotYx));
	__m128 ry = _mm_add_ps(_mm_mul_ps(ex, rotXy), _mm_mul_ps(ey, rotYy));

	ey = _mm_xor_ps(ey, _mm_set_ps1(-0.f));

	__m128 rx2 = _mm_add_ps(_mm_mul_ps(ex, rotXx), _mm_mul_ps(ey, rotYx));
	__m128 ry2 = _mm_add_ps(_mm_mul_ps(ex, rotXy), _mm_mul_ps(ey, rotYy));

	__m128 ax = _mm_shuffle_ps(rotX, rotX, _MM_SHUFFLE(1, 0, 3, 2));
	__m128 ay = _mm_shuffle_ps(rotY, rotY, _MM_SHUFFLE(1, 0, 3, 2));

	__m128 absMask = _mm_castsi128_ps(_mm_set1_epi32(0x7fffffff));
	__m128 r = _mm_and_ps(_mm_add_ps(_mm_mul_ps(rx, ax), _mm_mul_ps(ry, ay)), absMask);
	__m128 r2 = _mm_and_ps(_mm_add_ps(_mm_mul_ps(rx2, ax), _mm_mul_ps(ry2, ay)), absMask);

	__m128 rs = _mm_add_ps(_mm_max_ps(r, r2), e);

	__m128 d = _mm_and_ps(_mm_add_ps(_mm_mul_ps(dx, ax), _mm_mul_ps(dy, ay)), absMask);

	__m128 res = _mm_cmpgt_ps(d, rs);
	int resMask = _mm_movemask_ps(res);

	return resMask == 0;
}

//bool CPhysicEngine::SIMD_Set_OBBCollisionTest(CPolygonPtr p1, CPolygonPtr p2) const noexcept
//{
//	__m128 p1x = _mm_set_ps1(p1->position.x);
//	__m128 p1y = _mm_set_ps1(p1->position.y);
//
//	__m128 p2x = _mm_set_ps1(p2->position.x);
//	__m128 p2y = _mm_set_ps1(p2->position.y);
//
//	__m128 dx = _mm_sub_ps(p1x, p2x);
//	__m128 dy = _mm_sub_ps(p1y, p2y);
//
//	__m128 ex = _mm_set_ps(p1->halfExtent.x, p1->halfExtent.x, p2->halfExtent.x, p2->halfExtent.x);
//	__m128 ey = _mm_set_ps(p1->halfExtent.y, p1->halfExtent.y, p2->halfExtent.y, p2->halfExtent.y);
//
//
//	__m128 rotXx = _mm_set_ps(p1->rotation.X.x, p1->rotation.X.x, p2->rotation.X.x, p2->rotation.X.x);
//	__m128 rotXy = _mm_set_ps(p1->rotation.X.y, p1->rotation.X.y, p2->rotation.X.y, p2->rotation.X.y);
//	__m128 rotYx = _mm_set_ps(p1->rotation.Y.x, p1->rotation.Y.x, p2->rotation.Y.x, p2->rotation.Y.x);
//	__m128 rotYy = _mm_set_ps(p1->rotation.Y.y, p1->rotation.Y.y, p2->rotation.Y.y, p2->rotation.Y.y);
//
//	//__m128 rotX = _mm_set_ps(p1->rotation.X.x, p1->rotation.Y.x, p2->rotation.X.x, p2->rotation.Y.x);
//	//__m128 rotY = _mm_set_ps(p1->rotation.X.y, p1->rotation.Y.y, p2->rotation.X.y, p2->rotation.Y.y);
//
//	//__m128 rotXx = _mm_shuffle_ps(rotX, rotX, _MM_SHUFFLE(3, 3, 1, 1));
//	//__m128 rotXy = _mm_shuffle_ps(rotY, rotY, _MM_SHUFFLE(3, 3, 1, 1));
//	//__m128 rotYx = _mm_shuffle_ps(rotX, rotX, _MM_SHUFFLE(2, 2, 0, 0));
//	//__m128 rotYy = _mm_shuffle_ps(rotY, rotY, _MM_SHUFFLE(2, 2, 0, 0));
//
//	__m128 rx = _mm_add_ps(_mm_mul_ps(ex, rotXx), _mm_mul_ps(ey, rotYx));
//	__m128 ry = _mm_add_ps(_mm_mul_ps(ex, rotXy), _mm_mul_ps(ey, rotYy));
//
//	ey = _mm_xor_ps(ey, _mm_set_ps1(-0.f));
//
//	__m128 rx2 = _mm_add_ps(_mm_mul_ps(ex, rotXx), _mm_mul_ps(ey, rotYx));
//	__m128 ry2 = _mm_add_ps(_mm_mul_ps(ex, rotXy), _mm_mul_ps(ey, rotYy));
//
//	__m128 ax = _mm_set_ps(p2->rotation.X.x, p2->rotation.Y.x, p1->rotation.X.x, p1->rotation.Y.x);
//	__m128 ay = _mm_set_ps(p2->rotation.X.y, p2->rotation.Y.y, p1->rotation.X.y, p1->rotation.Y.y);
//	//__m128 ax = _mm_shuffle_ps(rotX, rotX, _MM_SHUFFLE(1, 0, 3, 2));
//	//__m128 ay = _mm_shuffle_ps(rotY, rotY, _MM_SHUFFLE(1, 0, 3, 2));
//
//	__m128 absMask = _mm_castsi128_ps(_mm_set1_epi32(0x7fffffff));
//	__m128 r = _mm_and_ps(_mm_add_ps(_mm_mul_ps(rx, ax), _mm_mul_ps(ry, ay)), absMask);
//	__m128 r2 = _mm_and_ps(_mm_add_ps(_mm_mul_ps(rx2, ax), _mm_mul_ps(ry2, ay)), absMask);
//	__m128 e = _mm_set_ps(p2->halfExtent.x, p2->halfExtent.y, p1->halfExtent.x, p1->halfExtent.y);
//	__m128 rs = _mm_add_ps(_mm_max_ps(r, r2), e);
//
//	__m128 d = _mm_and_ps(_mm_add_ps(_mm_mul_ps(dx, ax), _mm_mul_ps(dy, ay)), absMask);
//
//	__m128 res = _mm_cmpgt_ps(d, rs);
//	int resMask = _mm_movemask_ps(res);
//
//	return resMask == 0;
//}

//bool CPhysicEngine::SIMD_Set_Shuffle_OBBCollisionTest(CPolygonPtr p1, CPolygonPtr p2) const noexcept
//{
//	__m128 pos = _mm_set_ps(p2->position.y, p2->position.x, p1->position.y, p1->position.x);
//	__m128 extent = _mm_set_ps(p2->halfExtent.y, p2->halfExtent.x, p1->halfExtent.y, p1->halfExtent.x);
//	__m128 rotX = _mm_set_ps(p1->rotation.X.x, p1->rotation.Y.x, p2->rotation.X.x, p2->rotation.Y.x);
//	__m128 rotY = _mm_set_ps(p1->rotation.X.y, p1->rotation.Y.y, p2->rotation.X.y, p2->rotation.Y.y);
//
//	__m128 p1x = _mm_shuffle_ps(pos, pos, _MM_SHUFFLE(0, 0, 0, 0));
//	__m128 p1y = _mm_shuffle_ps(pos, pos, _MM_SHUFFLE(1, 1, 1, 1));
//	__m128 p2x = _mm_shuffle_ps(pos, pos, _MM_SHUFFLE(2, 2, 2, 2));
//	__m128 p2y = _mm_shuffle_ps(pos, pos, _MM_SHUFFLE(3, 3, 3, 3));
//
//	__m128 dx = _mm_sub_ps(p1x, p2x);
//	__m128 dy = _mm_sub_ps(p1y, p2y);
//
//	__m128 ex = _mm_shuffle_ps(extent, extent, _MM_SHUFFLE(0, 0, 2, 2));
//	__m128 ey = _mm_shuffle_ps(extent, extent, _MM_SHUFFLE(1, 1, 3, 3));
//
//	__m128 tmp_e = _mm_blend_ps(ex, ey, 0b1010);
//	__m128 e = _mm_shuffle_ps(tmp_e, tmp_e, _MM_SHUFFLE(0, 1, 2, 3));
//
//	__m128 rotXx = _mm_shuffle_ps(rotX, rotX, _MM_SHUFFLE(3, 3, 1, 1));
//	__m128 rotXy = _mm_shuffle_ps(rotY, rotY, _MM_SHUFFLE(3, 3, 1, 1));
//	__m128 rotYx = _mm_shuffle_ps(rotX, rotX, _MM_SHUFFLE(2, 2, 0, 0));
//	__m128 rotYy = _mm_shuffle_ps(rotY, rotY, _MM_SHUFFLE(2, 2, 0, 0));
//
//	__m128 rx = _mm_add_ps(_mm_mul_ps(ex, rotXx), _mm_mul_ps(ey, rotYx));
//	__m128 ry = _mm_add_ps(_mm_mul_ps(ex, rotXy), _mm_mul_ps(ey, rotYy));
//
//	ey = _mm_xor_ps(ey, _mm_set_ps1(-0.f));
//
//	__m128 rx2 = _mm_add_ps(_mm_mul_ps(ex, rotXx), _mm_mul_ps(ey, rotYx));
//	__m128 ry2 = _mm_add_ps(_mm_mul_ps(ex, rotXy), _mm_mul_ps(ey, rotYy));
//
//	__m128 ax = _mm_shuffle_ps(rotX, rotX, _MM_SHUFFLE(1, 0, 3, 2));
//	__m128 ay = _mm_shuffle_ps(rotY, rotY, _MM_SHUFFLE(1, 0, 3, 2));
//
//	__m128 absMask = _mm_castsi128_ps(_mm_set1_epi32(0x7fffffff));
//	__m128 r = _mm_and_ps(_mm_add_ps(_mm_mul_ps(rx, ax), _mm_mul_ps(ry, ay)), absMask);
//	__m128 r2 = _mm_and_ps(_mm_add_ps(_mm_mul_ps(rx2, ax), _mm_mul_ps(ry2, ay)), absMask);
//
//	__m128 rs = _mm_add_ps(_mm_max_ps(r, r2), e);
//
//	__m128 d = _mm_and_ps(_mm_add_ps(_mm_mul_ps(dx, ax), _mm_mul_ps(dy, ay)), absMask);
//
//	__m128 res = _mm_cmpgt_ps(d, rs);
//	int resMask = _mm_movemask_ps(res);
//
//	return resMask == 0;
//}

//bool CPhysicEngine::SISD_OBBCollisionTest(CPolygonPtr p1, CPolygonPtr p2) const noexcept
//{
//	float ra, rb;
//
//	float aE[2] = { p1->halfExtent.x, p1->halfExtent.y };
//	float bE[2] = { p2->halfExtent.x, p2->halfExtent.y };
//
//	float R[2][2];
//	float absR[2][2];
//
//	R[0][0] = p1->rotation.X | p2->rotation.X;
//	R[0][1] = p1->rotation.X | p2->rotation.Y;
//	R[1][0] = p1->rotation.Y | p2->rotation.X;
//	R[1][1] = p1->rotation.Y | p2->rotation.Y;
//
//	for (size_t i = 0; i < 2; i++)
//		for (size_t j = 0; j < 2; j++)
//			absR[i][j] = abs(R[i][j]) + FLT_EPSILON;
//
//	Vec2 tmp = p2->position - p1->position;
//	tmp = Vec2(tmp | p1->rotation.X, tmp | p1->rotation.Y);
//
//	float t[2] = { tmp.x, tmp.y };
//
//
//	//// Test axes L = A0, L = A1
//	for (size_t i = 0; i < 2; i++)
//	{
//		ra = aE[i];
//		rb = bE[0] * absR[i][0] + bE[1] * absR[i][1];
//
//		if (abs(t[i]) > ra + rb)
//			return 0;
//	}
//
//	// Test axes L = B0, L = B1
//	for (size_t i = 0; i < 2; i++)
//	{
//		ra = aE[0] * absR[0][i] + aE[1] * absR[1][i];
//		rb = bE[i];
//
//		if (abs(t[0] * R[0][i] + t[1] * R[1][i]) > ra + rb)
//			return 0;
//	}
//
//	return 1;
//}


void	CPhysicEngine::CollisionNarrowPhase()
{
	m_collidingPairs.clear();
	CPolygon& poly = gVars->pWorld->GetPolygons();

	for (const SPolygonPair& pair : m_pairsToCheck)
	{
		SCollision collision;
		size_t idx1 = pair.polyA;
		size_t idx2 = pair.polyB;
		
		const size_t arrayIdx1 = floor(idx1 / 4);
		const size_t registerIdx1 = idx1 % 4;
		const size_t arrayIdx2 = floor(idx2 / 4);
		const size_t registerIdx2 = idx2 % 4;
		
		constexpr int mask0 = _MM_SHUFFLE(0, 0, 0, 0);
		constexpr int mask1 = _MM_SHUFFLE(1, 1, 1, 1);
		constexpr int mask2 = _MM_SHUFFLE(2, 2, 2, 2);
		constexpr int mask3 = _MM_SHUFFLE(3, 3, 3, 3);
		constexpr int mask[4] = { mask0, mask1, mask2, mask3 };

		__m128 pos1, pos2, extent1, extent2;
		if (registerIdx1 == 0)
		{
			pos1 = _mm_shuffle_ps(poly.positionX[arrayIdx1], poly.positionY[arrayIdx1], mask0);
			extent1 = _mm_shuffle_ps(poly.halfExtentX[arrayIdx1], poly.halfExtentY[arrayIdx1], mask0);
		}
		else if (registerIdx1 == 1)
		{
			pos1 = _mm_shuffle_ps(poly.positionX[arrayIdx1], poly.positionY[arrayIdx1], mask1);
			extent1 = _mm_shuffle_ps(poly.halfExtentX[arrayIdx1], poly.halfExtentY[arrayIdx1], mask1);
		}
		else if (registerIdx1 == 2)
		{
			pos1 = _mm_shuffle_ps(poly.positionX[arrayIdx1], poly.positionY[arrayIdx1], mask2);
			extent1 = _mm_shuffle_ps(poly.halfExtentX[arrayIdx1], poly.halfExtentY[arrayIdx1], mask2);
		}
		else if (registerIdx1 == 3)
		{
			pos1 = _mm_shuffle_ps(poly.positionX[arrayIdx1], poly.positionY[arrayIdx1], mask3);
			extent1 = _mm_shuffle_ps(poly.halfExtentX[arrayIdx1], poly.halfExtentY[arrayIdx1], mask3);
		}

		if (registerIdx2 == 0)
		{
			pos2 = _mm_shuffle_ps(poly.positionX[arrayIdx2], poly.positionY[arrayIdx2], mask0);
			extent2 = _mm_shuffle_ps(poly.halfExtentX[arrayIdx2], poly.halfExtentY[arrayIdx2], mask0);
		}
		else if (registerIdx2 == 1)
		{
			pos2 = _mm_shuffle_ps(poly.positionX[arrayIdx2], poly.positionY[arrayIdx2], mask1);
			extent2 = _mm_shuffle_ps(poly.halfExtentX[arrayIdx2], poly.halfExtentY[arrayIdx2], mask1);
		}
		else if (registerIdx2 == 2)
		{
			pos2 = _mm_shuffle_ps(poly.positionX[arrayIdx2], poly.positionY[arrayIdx2], mask2);
			extent2 = _mm_shuffle_ps(poly.halfExtentX[arrayIdx2], poly.halfExtentY[arrayIdx2], mask2);
		}
		else if (registerIdx2 == 3)
		{
			pos2 = _mm_shuffle_ps(poly.positionX[arrayIdx2], poly.positionY[arrayIdx2], mask3);
			extent2 = _mm_shuffle_ps(poly.halfExtentX[arrayIdx2], poly.halfExtentY[arrayIdx2], mask3);
		}

		__m128 pos = _mm_shuffle_ps(pos1, pos2, _MM_SHUFFLE(2, 0, 2, 0));
		__m128 extent = _mm_shuffle_ps(extent1, extent2, _MM_SHUFFLE(2, 0, 2, 0));


		__m128 rotX = _mm_shuffle_ps(poly.registerRotation[idx2], poly.registerRotation[idx1], _MM_SHUFFLE(0, 2, 0, 2));
		__m128 rotY = _mm_shuffle_ps(poly.registerRotation[idx2], poly.registerRotation[idx1], _MM_SHUFFLE(1, 3, 1, 3));

		
		if (SIMD_Shuffle_OBBCollisionTest(pos, extent, rotX, rotY))
		{
			m_collidingPairs.push_back(collision);
		}
	}
}