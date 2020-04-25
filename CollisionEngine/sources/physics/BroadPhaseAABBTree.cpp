#include "physics\BroadPhaseAABBTree.h"

#include "GlobalVariables.h"
#include "World.h"

void CBroadPhaseAABBTree::GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck)
{
    size_t polyCount = gVars->pWorld->GetPolygonCount();
    const Node4* bvh4Nodes = gVars->pPhysicEngine->GetBVH4Nodes();

    for (size_t i = 0; i < polyCount; i++)
    {
        // Expand the AABB of the polygon we're going to test into a PackedAABB
        // so that it can be tested against the 4 AABBs in a BVH4 node at once
        PackedAABB polyAABBPacked(gVars->pPhysicEngine->GetWorldAABB(i));

        BVH4TraversalRecurse(i, polyAABBPacked, bvh4Nodes, 0, pairsToCheck);
    }
}

void CBroadPhaseAABBTree::BVH4TraversalRecurse(size_t polyIndex, const PackedAABB& polyAABBPacked, const Node4* bvh4Nodes, int32_t currentNodeIndex, std::vector<SPolygonPair>& pairsToCheck) const noexcept
{
    const Node4& node = bvh4Nodes[currentNodeIndex];
    // Overlap test between the AABB of the polygon and all the AABBs in the BVH4 node
    int collisionMask = PackedAABB::Intersect(polyAABBPacked, node.packedAABBs);

    // Check if the first AABB in the node has returned a hit
    if (collisionMask & 0b0001)
    {
        ChildID child = node.children[0];

        // If it's a leaf we have a potential collision with another polygon
        if (child.isLeaf)
        {
            // Don't add the pair if it's the polygone we're testing (child.index == polyIndex),
            // or one that has already been tested against the tree (childIndex < polyIndex) in
            // which case the potential collision has already been reported
            if (child.index > polyIndex)
                pairsToCheck.push_back(SPolygonPair(gVars->pWorld->GetPolygon(polyIndex), gVars->pWorld->GetPolygon(child.index)));
        }
        // If it's a node continue travelling down the tree
        else
            BVH4TraversalRecurse(polyIndex, polyAABBPacked, bvh4Nodes, child.index, pairsToCheck);
    }

    // Loop unrolling Agner Fog's style YAY \o/

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