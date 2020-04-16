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

void DrawTree(Node* tree, float index)
{
	if (tree != nullptr)
	{
		AABB::DrawWorld(tree->aabb);

		if (tree->isLeaf == false)
		{
			DrawTree(tree->leftNode, ++index);
			DrawTree(tree->rightNode, ++index);
		}
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
	BuildAABBTree_Internal(&tree, std::vector<AABB>(m_worldAABBs.begin() + 4, m_worldAABBs.end()));


	DrawTree(tree, 0);
	

	//AABB surrender(_mm_min_ps(m_worldAABBs[4].reg, m_worldAABBs[5].reg));
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

		std::vector<AABB> xSortedAABB, ySortedAABB;
		xSortedAABB = ySortedAABB = aabbs;

		std::sort(xSortedAABB.begin(), xSortedAABB.end(), [](AABB& a, AABB& b)
			{
				return ((a.minimum.x + a.maximum.x) * 0.5f) < ((b.minimum.x + b.maximum.x) * 0.5f);
			});

		std::sort(ySortedAABB.begin(), ySortedAABB.end(), [](AABB& a, AABB& b)
			{
				return ((a.minimum.y + a.maximum.y) * 0.5f) < ((b.minimum.y + b.maximum.y) * 0.5f);
			});

		const int middle = aabbCount / 2;

		std::vector<AABB> backX(std::make_move_iterator(xSortedAABB.cbegin()), std::make_move_iterator(xSortedAABB.cbegin() + middle));
		std::vector<AABB> frontX(std::make_move_iterator(xSortedAABB.cbegin() + middle), std::make_move_iterator(xSortedAABB.cend()));

		std::vector<AABB> backY(std::make_move_iterator(ySortedAABB.cbegin()), std::make_move_iterator(ySortedAABB.cbegin() + middle));
		std::vector<AABB> frontY(std::make_move_iterator(ySortedAABB.cbegin() + middle), std::make_move_iterator(ySortedAABB.cend()));


		float SAH_X = AABB::GetSurface(backX) + AABB::GetSurface(frontX);
		float SAH_Y = AABB::GetSurface(backY) + AABB::GetSurface(frontY);

//		if (gVars->pPhysicEngine->useSAH)
		{
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
		//else
		//{
		//	if (gVars->bDebug)
		//	{
		//		BuildAABBTree_Internal(&(node->leftNode), std::move(backY));
		//		BuildAABBTree_Internal(&(node->rightNode), std::move(frontY));
		//	}
		//	else
		//	{
		//		BuildAABBTree_Internal(&(node->leftNode), std::move(backX));
		//		BuildAABBTree_Internal(&(node->rightNode), std::move(frontX));
		//	}
		//}
	}
}

void	CPhysicEngine::CollisionBroadPhase()
{
	m_pairsToCheck.clear();
	m_broadPhase->GetCollidingPairsToCheck(m_pairsToCheck);
}

void	CPhysicEngine::CollisionNarrowPhase()
{
	m_collidingPairs.clear();
	for (const SPolygonPair& pair : m_pairsToCheck)
	{
		SCollision collision;
		collision.polyA = pair.polyA;
		collision.polyB = pair.polyB;
		if (pair.polyA->CheckCollision(*(pair.polyB), collision.point, collision.normal, collision.distance)) 
		{
			m_collidingPairs.push_back(collision);
		}
	}
}