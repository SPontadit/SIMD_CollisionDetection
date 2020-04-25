#include "physics\BroadPhaseAABBTree.h"

#include "GlobalVariables.h"
#include "World.h"

void CBroadPhaseAABBTree::GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck)
{
    size_t polyCount = gVars->pWorld->GetPolygonCount();
    const Node4* bvh4Nodes = gVars->pPhysicEngine->GetBVH4Nodes();

    for (size_t i = 0; i < polyCount; i++)
    {
        PackedAABB polyAABBPacked(gVars->pPhysicEngine->GetWorldAABB(i));

        BVH4TraversalRecurse(i, polyAABBPacked, bvh4Nodes, 0, pairsToCheck);
    }
}

void CBroadPhaseAABBTree::BVH4TraversalRecurse(size_t polyIndex, const PackedAABB& polyAABBPacked, const Node4* bvh4Nodes, int32_t currentNodeIndex, std::vector<SPolygonPair>& pairsToCheck) const noexcept
{
    const Node4& node = bvh4Nodes[currentNodeIndex];
    int collisionMask = PackedAABB::Intersect(polyAABBPacked, node.packedAABBs);

    if (collisionMask & 0b0001)
    {
        ChildID child = node.children[0];

        if (child.isLeaf)
        {
            if (child.index > polyIndex)
                pairsToCheck.push_back(SPolygonPair(gVars->pWorld->GetPolygon(polyIndex), gVars->pWorld->GetPolygon(child.index)));
        }
        else
            BVH4TraversalRecurse(polyIndex, polyAABBPacked, bvh4Nodes, child.index, pairsToCheck);
    }

    if (collisionMask & 0b0010)
    {
        ChildID child = node.children[1];

        if (child.isLeaf)
        {
            if (child.index > polyIndex)
                pairsToCheck.push_back(SPolygonPair(gVars->pWorld->GetPolygon(polyIndex), gVars->pWorld->GetPolygon(child.index)));
        }
        else
            BVH4TraversalRecurse(polyIndex, polyAABBPacked, bvh4Nodes, child.index, pairsToCheck);
    }

    if (collisionMask & 0b0100)
    {
        ChildID child = node.children[2];

        if (child.isLeaf)
        {
            if (child.index > polyIndex)
                pairsToCheck.push_back(SPolygonPair(gVars->pWorld->GetPolygon(polyIndex), gVars->pWorld->GetPolygon(child.index)));
        }
        else
            BVH4TraversalRecurse(polyIndex, polyAABBPacked, bvh4Nodes, child.index, pairsToCheck);
    }

    if (collisionMask & 0b1000)
    {
        ChildID child = node.children[3];

        if (child.isLeaf)
        {
            if (child.index > polyIndex)
                pairsToCheck.push_back(SPolygonPair(gVars->pWorld->GetPolygon(polyIndex), gVars->pWorld->GetPolygon(child.index)));
        }
        else
            BVH4TraversalRecurse(polyIndex, polyAABBPacked, bvh4Nodes, child.index, pairsToCheck);
    }
}