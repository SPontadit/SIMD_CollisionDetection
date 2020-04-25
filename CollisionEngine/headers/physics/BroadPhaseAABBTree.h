#ifndef _BROAD_PHASE_AABB_TREE_H_
#define _BROAD_PHASE_AABB_TREE_H_

#include "BroadPhase.h"

class CBroadPhaseAABBTree : public IBroadPhase
{
public:
    virtual void GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck) override;

private:
    void BVH4TraversalRecurse(size_t polyIndex, const PackedAABB& polyAABBPacked, const Node4* bvh4Nodes, int32_t currentNodeIndex, std::vector<SPolygonPair>& pairsToCheck) const noexcept;
};

#endif