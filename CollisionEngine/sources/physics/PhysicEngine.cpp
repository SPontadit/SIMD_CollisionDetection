#include "physics/PhysicEngine.h"

#include <iostream>
#include <string>
#include <algorithm>
#include "GlobalVariables.h"
#include "World.h"
#include "render/Renderer.h" // for debugging only
#include "Timer.h"

#include "physics/BroadPhase.h"
#include "physics/BroadPhaseBrut.h"


void	CPhysicEngine::Reset()
{
	m_pairsToCheck.clear();
	m_collidingPairs.clear();

	m_localAABBs.clear();
	m_worldAABBs.clear();

	m_active = true;

	m_broadPhase = new CBroadPhaseBrut();
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

void DrawTree(Node* tree)
{
	if (tree != nullptr)
	{
		AABB::DrawWorld(tree->aabb);

		if (tree->isLeaf == false)
		{
			DrawTree(tree->leftNode);
			DrawTree(tree->rightNode);
		}
	}
}

void DrawTree4(Node4* node)
{
	if (node == nullptr)
		return;

	for (size_t i = 0; i < 4; i++)
	{
		AABB::DrawWorld(node->GetAABB(i));

		Node4* child = node->children[i];
		if (child != nullptr)
			DrawTree4(child);
	}
}

void	CPhysicEngine::BuildAABBTree()
{
	m_worldAABBs.clear();

	const int aabbSize = m_localAABBs.size();


	for (size_t i = 0; i < aabbSize; i++)
	{
		CPolygonPtr poly = gVars->pWorld->GetPolygon(i);

		AABB worldAABB = m_localAABBs[i].Transform(poly->position, poly->rotation);
		m_worldAABBs.push_back(worldAABB);

		//AABB::DrawWorld(m_worldAABBs[i]);
	}

	Node* tree = new Node();

	// Reuse pointer ?
	BuildAABBTree_Internal(&tree, std::vector<AABB>(m_worldAABBs.begin(), m_worldAABBs.end()));

	// Convert BVH2 to BVH4
	Node4* tree4 = BVH2ToBVH4(tree);

	DrawTree4(tree4);
	

	/*AABB surrender(_mm_min_ps(m_worldAABBs[0].reg, m_worldAABBs[1].reg));


	__m128 minX = _mm_set_ps( m_worldAABBs[0].minimum.x,  m_worldAABBs[1].minimum.x,  m_worldAABBs[2].minimum.x,  m_worldAABBs[3].minimum.x);
	__m128 minY = _mm_set_ps( m_worldAABBs[0].minimum.y,  m_worldAABBs[1].minimum.y,  m_worldAABBs[2].minimum.y,  m_worldAABBs[3].minimum.y);
	__m128 maxX = _mm_set_ps(-m_worldAABBs[0].maximum.x, -m_worldAABBs[1].maximum.x, -m_worldAABBs[2].maximum.x, -m_worldAABBs[3].maximum.x);
	__m128 maxY = _mm_set_ps(-m_worldAABBs[0].maximum.y, -m_worldAABBs[1].maximum.y, -m_worldAABBs[2].maximum.y, -m_worldAABBs[3].maximum.y);

	__m128 testMinX = _mm_set_ps1( m_worldAABBs[4].minimum.x);
	__m128 testMinY = _mm_set_ps1( m_worldAABBs[4].minimum.y);
	__m128 testMaxX = _mm_set_ps1(-m_worldAABBs[4].maximum.x);
	__m128 testMaxY = _mm_set_ps1(-m_worldAABBs[4].maximum.y);

	PackedAABB node = PackedAABB(minX, minY, maxX, maxY);
	PackedAABB test = PackedAABB(testMinX, testMinY, testMaxX, testMaxY);

	std::cout << AABB::Intersect(test, node) << std::endl;*/
}

void CPhysicEngine::BuildAABBTree_Internal(Node** tree, std::vector<AABB>& aabbs)
{
	size_t aabbCount = aabbs.size();

	Node* node = new Node();
	*tree = node;

	node->aabb = AABB::GetSurrounding(aabbs);

	if (aabbCount <= 1)
	{
		node->isLeaf = true;
		return;
	}
	else
	{
		node->isLeaf = false;

		std::vector<AABB> xSortedAABB(std::move(aabbs));
		std::vector<AABB> ySortedAABB(xSortedAABB);
		AABB* xSort, ySort;

		std::sort(xSortedAABB.begin(), xSortedAABB.end(), [](AABB& a, AABB& b)
			{
				return ((a.minimum.x - a.maximum.x) * 0.5f) < ((b.minimum.x - b.maximum.x) * 0.5f);
			});

		std::sort(ySortedAABB.begin(), ySortedAABB.end(), [](AABB& a, AABB& b)
			{
				return ((a.minimum.y - a.maximum.y) * 0.5f) < ((b.minimum.y - b.maximum.y) * 0.5f);
			});

		const int middle = aabbCount / 2;

		std::vector<AABB> backX(std::make_move_iterator(xSortedAABB.cbegin()), std::make_move_iterator(xSortedAABB.cbegin() + middle));
		std::vector<AABB> frontX(std::make_move_iterator(xSortedAABB.cbegin() + middle), std::make_move_iterator(xSortedAABB.cend()));

		std::vector<AABB> backY(std::make_move_iterator(ySortedAABB.cbegin()), std::make_move_iterator(ySortedAABB.cbegin() + middle));
		std::vector<AABB> frontY(std::make_move_iterator(ySortedAABB.cbegin() + middle), std::make_move_iterator(ySortedAABB.cend()));


		float SAH_X = AABB::GetSurface(backX) + AABB::GetSurface(frontX);
		float SAH_Y = AABB::GetSurface(backY) + AABB::GetSurface(frontY);

		if (SAH_X < SAH_Y)
		{
			BuildAABBTree_Internal(&(node->leftNode), std::move(backX));
			BuildAABBTree_Internal(&(node->rightNode), std::move(frontX));
		}
		else
		{
			BuildAABBTree_Internal(&(node->leftNode), std::move(backY));
			BuildAABBTree_Internal(&(node->rightNode), std::move(frontY));
		}
	}
}

Node4* CPhysicEngine::BVH2ToBVH4(Node* node2)
{
	if (node2->isLeaf)
		return nullptr;

	Node4* node4 = new Node4;

	Node* nodes[4];

	nodes[0] = node2->leftNode;
	node4->SetAABB(0, node2->leftNode->aabb);

	nodes[1] = node2->rightNode;
	node4->SetAABB(1, node2->rightNode->aabb);

	size_t childCount = 2;

	while (childCount < 4)
	{
		int64_t bestIdx = -1;
		float bestArea = -std::numeric_limits<float>::infinity();

		for (size_t i = 0; i < childCount; i++)
		{
			if (nodes[i]->isLeaf)
				continue;

			float area = nodes[i]->aabb.Surface();
			if (area > bestArea)
			{
				bestArea = area;
				bestIdx = i;
			}
		}

		if (bestIdx < 0)
			break;

		Node* bestNode = nodes[bestIdx];
		nodes[bestIdx] = bestNode->leftNode;
		node4->SetAABB(bestIdx, bestNode->leftNode->aabb);
		nodes[childCount] = bestNode->rightNode;
		node4->SetAABB(childCount, bestNode->rightNode->aabb);

		childCount++;
	}

	for (size_t i = 0; i < childCount; i++)
		node4->children[i] = BVH2ToBVH4(nodes[i]);

	return node4;
}

void	CPhysicEngine::CollisionBroadPhase()
{
	m_pairsToCheck.clear();
	m_broadPhase->GetCollidingPairsToCheck(m_pairsToCheck);
}

bool CPhysicEngine::SIMD_OBBCollisionTest(CPolygonPtr p1, CPolygonPtr p2) const noexcept
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

	__m128 rx = _mm_add_ps(_mm_mul_ps(ex, rotXx), _mm_mul_ps(ey, rotYx));
	__m128 ry = _mm_add_ps(_mm_mul_ps(ex, rotXy), _mm_mul_ps(ey, rotYy));

	ey = _mm_xor_ps(ey, _mm_set_ps1(-0.f));

	__m128 rx2 = _mm_add_ps(_mm_mul_ps(ex, rotXx), _mm_mul_ps(ey, rotYx));
	__m128 ry2 = _mm_add_ps(_mm_mul_ps(ex, rotXy), _mm_mul_ps(ey, rotYy));

	__m128 ax = _mm_set_ps(p2->rotation.X.x, p2->rotation.Y.x, p1->rotation.X.x, p1->rotation.Y.x);
	__m128 ay = _mm_set_ps(p2->rotation.X.y, p2->rotation.Y.y, p1->rotation.X.y, p1->rotation.Y.y);

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
		//if (pair.polyA->CheckCollision(*(pair.polyB), collision.point, collision.normal, collision.distance)) 
		if (SIMD_OBBCollisionTest(pair.polyA, pair.polyB))
		{
			m_collidingPairs.push_back(collision);
		}
	}
}