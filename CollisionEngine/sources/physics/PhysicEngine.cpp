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
	// We use the std::vector class for easy memory management but pass pointers to the
	// BVH construction functions to use the lighter pointer syntax compared to iterators

	const size_t objectCount = m_localAABBs.size();

	// Contains the world AABBs of all polygons in the same order as the polygons themselves
	m_worldAABBs.resize(objectCount);

	// Contain Leaf structures, they store a world AABB and the index of the polygon it belongs to
	m_xSortedLeaves.resize(objectCount);
	m_ySortedLeaves.resize(objectCount);

	for (size_t i = 0; i < objectCount; i++)
	{
		CPolygonPtr poly = gVars->pWorld->GetPolygon(i);

		AABB worldAABB = m_localAABBs[i].Transform(poly->position, poly->rotation);

		m_worldAABBs[i] = worldAABB;
		m_ySortedLeaves[i] = m_xSortedLeaves[i] = Leaf(worldAABB, i);
	}

	// The tree doesn't contains the leaves so the number of nodes is number of leaves -1
	size_t nodeCount = objectCount  - 1;
	m_bvh2Nodes.resize(nodeCount);
	m_bvh4Nodes.resize(nodeCount);

	// Build BVH2
	int32_t newNodeIndex = 0;
	BVH2Recurse(m_bvh2Nodes.data(), newNodeIndex, m_xSortedLeaves.data(), m_ySortedLeaves.data(), objectCount);

	//if (gVars->bDebug)
	//	DrawBVH2(bvh2Nodes, nodeCount);

	// Build BVH4 from BVH2
	newNodeIndex = 0;
	BVH2ToBVH4(m_bvh2Nodes.data(), 0, m_bvh4Nodes.data(), newNodeIndex);

	if (gVars->bDebug)
		DrawBVH4(m_bvh4Nodes.data(), newNodeIndex);
}

int32_t CPhysicEngine::BVH2Recurse(Node2* nodes, int32_t& newNodeIndex, Leaf* xSortedLeaves, Leaf* ySortedLeaves, size_t leafCount)
{
	// Get index for the new node and increment index for recursive calls
	int32_t nodeIndex = newNodeIndex++;
	// Get pointer to the node we add to the tree
	Node2* node = nodes + nodeIndex;

	// Sort leaves depending on their center position along the X axis
	std::qsort(xSortedLeaves, leafCount, sizeof(Leaf), Leaf::SortCenterX);
	// Sort leaves depending on their center position along the Y axis
	std::qsort(ySortedLeaves, leafCount, sizeof(Leaf), Leaf::SortCenterY);

	const int frontCount = leafCount / 2;
	const int backCount = leafCount - frontCount;

	// Compute AABBs surrounding each half of the sorted leaves along both axes
	const AABB xFront = Leaf::GetSurroundingAABB(xSortedLeaves, frontCount);
	const AABB xBack = Leaf::GetSurroundingAABB(xSortedLeaves + frontCount, backCount);
	const AABB yFront = Leaf::GetSurroundingAABB(ySortedLeaves, frontCount);
	const AABB yBack = Leaf::GetSurroundingAABB(ySortedLeaves + frontCount, backCount);

	// Compute SAH cost for each split
	const float xSAH = xFront.Surface() + xBack.Surface();
	const float ySAH = yFront.Surface() + yBack.Surface();

	// Chopse the split with the minimum SAH cost <=> the split
	// that creates the smallest bounding areas for each half
	if (xSAH < ySAH)
	{
		// Copy the sorted leaves along the chosen axis to the other list
		// so that recursive calls get the correct halves on both axes
		memcpy(ySortedLeaves, xSortedLeaves, leafCount * sizeof(Leaf));

		// Store the AABBs of child boxes in the parent node
		node->childAABBs[0] = xFront;
		node->childAABBs[1] = xBack;

		// If the front half contains more than one leaf, recurse to create child nodes ...
		if (frontCount > 1)
		{
			node->children[0].index = BVH2Recurse(nodes, newNodeIndex, xSortedLeaves, ySortedLeaves, frontCount);
			node->children[0].isLeaf = false;
		}
		// ... otherwise don't recurse and link to child polygon with the index in the Leaf struct
		else
		{
			node->children[0].index = xSortedLeaves->polyIndex;
			node->children[0].isLeaf = true;
		}

		// Same for the back half
		if (backCount > 1)
		{
			node->children[1].index = BVH2Recurse(nodes, newNodeIndex, xSortedLeaves + frontCount, ySortedLeaves + frontCount, backCount);
			node->children[1].isLeaf = false;
		}
		else
		{
			node->children[1].index = xSortedLeaves[frontCount].polyIndex;
			node->children[1].isLeaf = true;
		}
	}
	else
	{
		// Same for the other axis

		memcpy(xSortedLeaves, ySortedLeaves, leafCount * sizeof(Leaf));

		node->childAABBs[0] = yFront;
		node->childAABBs[1] = yBack;

		if (frontCount > 1)
		{
			node->children[0].index = BVH2Recurse(nodes, newNodeIndex, xSortedLeaves, ySortedLeaves, frontCount);
			node->children[0].isLeaf = false;
		}
		else
		{
			node->children[0].index = ySortedLeaves->polyIndex;
			node->children[0].isLeaf = true;
		}

		if (backCount > 1)
		{
			node->children[1].index = BVH2Recurse(nodes, newNodeIndex, xSortedLeaves + frontCount, ySortedLeaves + frontCount, backCount);
			node->children[1].isLeaf = false;
		}
		else
		{
			node->children[1].index = ySortedLeaves[frontCount].polyIndex;
			node->children[1].isLeaf = true;
		}
	}

	// Return the index to the node we created, parent call will store it in its child index
	return nodeIndex;
}

int32_t CPhysicEngine::BVH2ToBVH4(Node2* bvh2Nodes, int32_t currentNode2Index, Node4* bvh4Nodes, int32_t& newNode4Index)
{
	// Get index for the new node and increment index for recursive calls
	int32_t node4Index = newNode4Index++;
	// We use a placement new here to reset the data that was in the node
	// We do this because contrary to the BVH2 we might not have all children set
	Node4* node4 = new(bvh4Nodes + node4Index) Node4;

	// Get a pointer to the BVH2 node we are working on
	Node2* node2 = bvh2Nodes + currentNode2Index;

	// Store the area values of the child AABBs here so we don't have to extract
	// the AABBs from the PackedAABB structure every time we need to compute an area
	float areas[4];

	// Initialize the first two children of the BVH4 node with the children of the BVH2 node
	node4->children[0] = node2->children[0];
	areas[0] = node2->childAABBs[0].Surface();
	node4->SetAABB(0, node2->childAABBs[0]);

	node4->children[1] = node2->children[1];
	areas[1] = node2->childAABBs[0].Surface();
	node4->SetAABB(1, node2->childAABBs[1]);

	size_t childCount = 2;

	// Try to grab as much children as possible from the hierarchy
	while (childCount < 4)
	{
		int64_t bestIdx = -1;
		float bestArea = -std::numeric_limits<float>::infinity();

		for (size_t i = 0; i < childCount; i++)
		{
			// Skip leaves, we can't get more children from them
			if (node4->children[i].isLeaf)
				continue;

			// Compare areas of children already under the BVH4 node, we want to get the largest
			// one and get its children so that the ratio between the area of the parent AABB and
			// the sum of the areas of the children AABBs is as large as possible (ie. more empty
			// space inside an BVH node so that overlap test may fail earlier)
			float area = areas[i];
			if (area > bestArea)
			{
				bestArea = area;
				bestIdx = i;
			}
		}

		// If there were only leaves in the children of the BVH4 node, bestIdx is still -1 and we
		// can't reach deeper in the BVH2 hierarchy so exit here
		if (bestIdx < 0)
			break;

		// Get the BVH2 node we chose with the largest area from the children already set ...
		Node2* bestNode = bvh2Nodes + node4->children[bestIdx].index;

		// ... and replace it with its first child
		node4->children[bestIdx] = bestNode->children[0];
		areas[bestIdx] = bestNode->childAABBs[0].Surface();
		node4->SetAABB(bestIdx, bestNode->childAABBs[0]);

		// The second child is a new child for the BVH4 node
		node4->children[childCount] = bestNode->children[1];
		areas[childCount] = bestNode->childAABBs[1].Surface();
		node4->SetAABB(childCount, bestNode->childAABBs[1]);

		childCount++;
	}

	// Recurse on children that are not leaves, replacing the index they hold that corresponds
	// to a BVH2 node with the index returned by this function for a BVH4 node
	for (size_t i = 0; i < childCount; i++)
	{
		if (!node4->children[i].isLeaf)
			node4->children[i].index = BVH2ToBVH4(bvh2Nodes, node4->children[i].index, bvh4Nodes, newNode4Index);
	}

	// Leaves children are left untouched, their index is still valid because it points to a polygon

	return node4Index;
}

void	CPhysicEngine::CollisionBroadPhase()
{
	m_pairsToCheck.clear();
	m_broadPhase->GetCollidingPairsToCheck(m_pairsToCheck);
}

bool CPhysicEngine::SIMD_Shuffle_OBBCollisionTest(__m128 pos, __m128 extent, __m128 rotXxYx, __m128 rotXyYy) const noexcept
{
	/*
	* This OBB-OBB overlapp test is derived from the algorithm described in
	* "Real Time Collision Detection" by Christer Ericson. It is a variation
	* of the SAT algorithm with simplifications allowed by the rectangular
	* nature of OBBs:
	* 	- Instead of computing a normal for each hedge of the box we use the
	* 	  vector base of the OBB coordinate frame which are contained in their
		  rotation matrices
	* 	- Because two opposite side of a rectangle are parallel, their
	* 	  normals are colinear and projections on them are identical in
	* 	  absolute value, so we can test only two axes per OBB
	* 	- Projections of an OBB on its vector bases are equal to its
	* 	  extents so we don't need to compute these
	*/

	// pos = { poly1.pos.x, poly1.pos.y, poly2.pos.x, poly2.pos.y }
	// extent = { poly1.extent.x, poly1.extent.y, poly2.extent.x, poly2.extent.y }
	// rotXxYx = { poly2.rot.X.x, poly2.rot.Y.x, poly1.rot.X.x, poly1.rot.Y.x }
	// rotXyYy = { poly2.rot.X.y, poly2.rot.Y.y, poly1.rot.X.y, poly1.rot.Y.y }

	// We splat each position component in a single register
	__m128 p1x = _mm_shuffle_ps(pos, pos, _MM_SHUFFLE(0, 0, 0, 0));
	__m128 p1y = _mm_shuffle_ps(pos, pos, _MM_SHUFFLE(1, 1, 1, 1));
	__m128 p2x = _mm_shuffle_ps(pos, pos, _MM_SHUFFLE(2, 2, 2, 2));
	__m128 p2y = _mm_shuffle_ps(pos, pos, _MM_SHUFFLE(3, 3, 3, 3));

	// We compute the translation between the two polygons centers
	__m128 tx = _mm_sub_ps(p1x, p2x);
	__m128 ty = _mm_sub_ps(p1y, p2y);

	// ex = { poly2.extent.x, poly2.extent.x, poly1.extent.x, poly1.extent.x }
	// ey = { poly2.extent.y, poly2.extent.y, poly1.extent.y, poly1.extent.y }
	__m128 ex = _mm_shuffle_ps(extent, extent, _MM_SHUFFLE(0, 0, 2, 2));
	__m128 ey = _mm_shuffle_ps(extent, extent, _MM_SHUFFLE(1, 1, 3, 3));

	// e = { poly1.extent.y, poly1.extent.x, poly2.extent.y, poly2.extent.x }
	__m128 e = _mm_blend_ps(ex, ey, 0b1010);
	e = _mm_shuffle_ps(e, e, _MM_SHUFFLE(0, 1, 2, 3));
	
	__m128 rotXx = _mm_shuffle_ps(rotXxYx, rotXxYx, _MM_SHUFFLE(1, 1, 3, 3));
	__m128 rotXy = _mm_shuffle_ps(rotXyYy, rotXyYy, _MM_SHUFFLE(1, 1, 3, 3));
	__m128 rotYx = _mm_shuffle_ps(rotXxYx, rotXxYx, _MM_SHUFFLE(0, 0, 2, 2));
	__m128 rotYy = _mm_shuffle_ps(rotXyYy, rotXyYy, _MM_SHUFFLE(0, 0, 2, 2));

	// rx and ry contain the extents of the 2 OBBs transformed in world space
	__m128 rx = _mm_add_ps(_mm_mul_ps(ex, rotXx), _mm_mul_ps(ey, rotYx));
	__m128 ry = _mm_add_ps(_mm_mul_ps(ex, rotXy), _mm_mul_ps(ey, rotYy));

	// Negate the y value to transform the extent (x, -y) in world space and project it too
	// because in some configuration the (x, y) extent won't give the maximum projection in
	// world space. Ericson's algorithm doesn't need that because he computes everything in
	// the coordinate frame of one of the first OBB and has a absolute value of the rotation
	// matrix to transform the second OBB into the coordinate frame oif the first one. The
	// transformed extents of an OBB in the other's coordinate frame using that rotation
	// matrix automatically produces a vector that will have the maximum projection on the
	// target vector base.
	ey = _mm_xor_ps(ey, _mm_set_ps1(-0.f));

	// rx2 and ry2 contain the (x, -y) extents transformed in world space
	__m128 rx2 = _mm_add_ps(_mm_mul_ps(ex, rotXx), _mm_mul_ps(ey, rotYx));
	__m128 ry2 = _mm_add_ps(_mm_mul_ps(ex, rotXy), _mm_mul_ps(ey, rotYy));

	// ax and ay contain the axes against which we will project the extents of the boxes
	__m128 ax = rotXxYx;
	__m128 ay = rotXyYy;

	__m128 absMask = _mm_castsi128_ps(_mm_set1_epi32(0x7fffffff));
	// Absolute value of dot product between the extents and the and the axes to find the projection
	__m128 r = _mm_and_ps(_mm_add_ps(_mm_mul_ps(rx, ax), _mm_mul_ps(ry, ay)), absMask);
	// Same with the (x, -y) extents
	__m128 r2 = _mm_and_ps(_mm_add_ps(_mm_mul_ps(rx2, ax), _mm_mul_ps(ry2, ay)), absMask);

	// Sum the maximum projection of one OBB on the vector base of the other
	// with the extents of the other OBB
	__m128 rs = _mm_add_ps(_mm_max_ps(r, r2), e);

	// Compute the projection of the translation vector on the vector bases of the OBBs
	__m128 t = _mm_and_ps(_mm_add_ps(_mm_mul_ps(tx, ax), _mm_mul_ps(ty, ay)), absMask);

	// Compare the projection of the translation vector with the sum of the projections
	// of the OBBs. If it is greater, then we have found a seeparating axis and the resMask
	// will contain et bit set to 1 for this axis
	__m128 res = _mm_cmpgt_ps(t, rs);
	int resMask = _mm_movemask_ps(res);

	// If no bit set in the mask we had no separatring axis found so the OBBs overlap !
	return resMask == 0;
}

bool CPhysicEngine::SIMD_Set_OBBCollisionTest(CPolygonPtr p1, CPolygonPtr p2) const noexcept
{
	__m128 p1x = _mm_set_ps1(p1->position.x);
	__m128 p1y = _mm_set_ps1(p1->position.y);

	__m128 p2x = _mm_set_ps1(p2->position.x);
	__m128 p2y = _mm_set_ps1(p2->position.y);

	__m128 dx = _mm_sub_ps(p1x, p2x);
	__m128 dy = _mm_sub_ps(p1y, p2y);

	__m128 ex = _mm_set_ps(p1->halfExtent.x, p1->halfExtent.x, p2->halfExtent.x, p2->halfExtent.x);
	__m128 ey = _mm_set_ps(p1->halfExtent.y, p1->halfExtent.y, p2->halfExtent.y, p2->halfExtent.y);


	__m128 rotXx = _mm_set_ps(p1->rotation.X.x, p1->rotation.X.x, p2->rotation.X.x, p2->rotation.X.x);
	__m128 rotXy = _mm_set_ps(p1->rotation.X.y, p1->rotation.X.y, p2->rotation.X.y, p2->rotation.X.y);
	__m128 rotYx = _mm_set_ps(p1->rotation.Y.x, p1->rotation.Y.x, p2->rotation.Y.x, p2->rotation.Y.x);
	__m128 rotYy = _mm_set_ps(p1->rotation.Y.y, p1->rotation.Y.y, p2->rotation.Y.y, p2->rotation.Y.y);

	//__m128 rotX = _mm_set_ps(p1->rotation.X.x, p1->rotation.Y.x, p2->rotation.X.x, p2->rotation.Y.x);
	//__m128 rotY = _mm_set_ps(p1->rotation.X.y, p1->rotation.Y.y, p2->rotation.X.y, p2->rotation.Y.y);

	//__m128 rotXx = _mm_shuffle_ps(rotX, rotX, _MM_SHUFFLE(3, 3, 1, 1));
	//__m128 rotXy = _mm_shuffle_ps(rotY, rotY, _MM_SHUFFLE(3, 3, 1, 1));
	//__m128 rotYx = _mm_shuffle_ps(rotX, rotX, _MM_SHUFFLE(2, 2, 0, 0));
	//__m128 rotYy = _mm_shuffle_ps(rotY, rotY, _MM_SHUFFLE(2, 2, 0, 0));

	__m128 rx = _mm_add_ps(_mm_mul_ps(ex, rotXx), _mm_mul_ps(ey, rotYx));
	__m128 ry = _mm_add_ps(_mm_mul_ps(ex, rotXy), _mm_mul_ps(ey, rotYy));

	ey = _mm_xor_ps(ey, _mm_set_ps1(-0.f));

	__m128 rx2 = _mm_add_ps(_mm_mul_ps(ex, rotXx), _mm_mul_ps(ey, rotYx));
	__m128 ry2 = _mm_add_ps(_mm_mul_ps(ex, rotXy), _mm_mul_ps(ey, rotYy));

	__m128 ax = _mm_set_ps(p2->rotation.X.x, p2->rotation.Y.x, p1->rotation.X.x, p1->rotation.Y.x);
	__m128 ay = _mm_set_ps(p2->rotation.X.y, p2->rotation.Y.y, p1->rotation.X.y, p1->rotation.Y.y);
	//__m128 ax = _mm_shuffle_ps(rotX, rotX, _MM_SHUFFLE(1, 0, 3, 2));
	//__m128 ay = _mm_shuffle_ps(rotY, rotY, _MM_SHUFFLE(1, 0, 3, 2));

	__m128 absMask = _mm_castsi128_ps(_mm_set1_epi32(0x7fffffff));
	__m128 r = _mm_and_ps(_mm_add_ps(_mm_mul_ps(rx, ax), _mm_mul_ps(ry, ay)), absMask);
	__m128 r2 = _mm_and_ps(_mm_add_ps(_mm_mul_ps(rx2, ax), _mm_mul_ps(ry2, ay)), absMask);
	__m128 e = _mm_set_ps(p2->halfExtent.x, p2->halfExtent.y, p1->halfExtent.x, p1->halfExtent.y);
	__m128 rs = _mm_add_ps(_mm_max_ps(r, r2), e);

	__m128 d = _mm_and_ps(_mm_add_ps(_mm_mul_ps(dx, ax), _mm_mul_ps(dy, ay)), absMask);

	__m128 res = _mm_cmpgt_ps(d, rs);
	int resMask = _mm_movemask_ps(res);

	return resMask == 0;
}

bool CPhysicEngine::SIMD_Set_Shuffle_OBBCollisionTest(CPolygonPtr p1, CPolygonPtr p2) const noexcept
{
	__m128 pos = _mm_set_ps(p2->position.y, p2->position.x, p1->position.y, p1->position.x);
	__m128 extent = _mm_set_ps(p2->halfExtent.y, p2->halfExtent.x, p1->halfExtent.y, p1->halfExtent.x);
	__m128 rotX = _mm_set_ps(p1->rotation.X.x, p1->rotation.Y.x, p2->rotation.X.x, p2->rotation.Y.x);
	__m128 rotY = _mm_set_ps(p1->rotation.X.y, p1->rotation.Y.y, p2->rotation.X.y, p2->rotation.Y.y);

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

bool CPhysicEngine::SISD_OBBCollisionTest(CPolygonPtr p1, CPolygonPtr p2) const noexcept
{
	float ra, rb;

	float aE[2] = { p1->halfExtent.x, p1->halfExtent.y };
	float bE[2] = { p2->halfExtent.x, p2->halfExtent.y };

	float R[2][2];
	float absR[2][2];

	R[0][0] = p1->rotation.X | p2->rotation.X;
	R[0][1] = p1->rotation.X | p2->rotation.Y;
	R[1][0] = p1->rotation.Y | p2->rotation.X;
	R[1][1] = p1->rotation.Y | p2->rotation.Y;

	for (size_t i = 0; i < 2; i++)
		for (size_t j = 0; j < 2; j++)
			absR[i][j] = abs(R[i][j]) + FLT_EPSILON;

	Vec2 tmp = p2->position - p1->position;
	tmp = Vec2(tmp | p1->rotation.X, tmp | p1->rotation.Y);

	float t[2] = { tmp.x, tmp.y };


	//// Test axes L = A0, L = A1
	for (size_t i = 0; i < 2; i++)
	{
		ra = aE[i];
		rb = bE[0] * absR[i][0] + bE[1] * absR[i][1];

		if (abs(t[i]) > ra + rb)
			return 0;
	}

	// Test axes L = B0, L = B1
	for (size_t i = 0; i < 2; i++)
	{
		ra = aE[0] * absR[0][i] + aE[1] * absR[1][i];
		rb = bE[i];

		if (abs(t[0] * R[0][i] + t[1] * R[1][i]) > ra + rb)
			return 0;
	}

	return 1;
}

void	CPhysicEngine::CollisionNarrowPhase()
{
	m_collidingPairs.clear();
	for (const SPolygonPair& pair : m_pairsToCheck)
	{
		SCollision collision;
		collision.polyA = pair.polyA;
		collision.polyB = pair.polyB;

		__m128 pos = _mm_set_ps(pair.polyB->position.y, pair.polyB->position.x, pair.polyA->position.y, pair.polyA->position.x);
		__m128 extent = _mm_set_ps(pair.polyB->halfExtent.y, pair.polyB->halfExtent.x, pair.polyA->halfExtent.y, pair.polyA->halfExtent.x);
		__m128 rotX = _mm_set_ps(pair.polyB->rotation.X.x, pair.polyB->rotation.Y.x, pair.polyA->rotation.X.x, pair.polyA->rotation.Y.x);
		__m128 rotY = _mm_set_ps(pair.polyB->rotation.X.y, pair.polyB->rotation.Y.y, pair.polyA->rotation.X.y, pair.polyA->rotation.Y.y);
		
		if (SIMD_Shuffle_OBBCollisionTest(pos, extent, rotX, rotY))
		{
			m_collidingPairs.push_back(collision);
		}
		if (SIMD_Set_OBBCollisionTest(pair.polyA, pair.polyB))
		{
			m_collidingPairs.push_back(collision);
		}
		if (SIMD_Set_Shuffle_OBBCollisionTest(pair.polyA, pair.polyB))
		{
			m_collidingPairs.push_back(collision);
		}
		if (SISD_OBBCollisionTest(pair.polyA, pair.polyB))
		{
			m_collidingPairs.push_back(collision);
		}
	}
}