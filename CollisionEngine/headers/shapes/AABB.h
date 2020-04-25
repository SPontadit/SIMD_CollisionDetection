#ifndef _AABB_H_
#define _AABB_H_

#include <cfloat>
#include "Maths.h"
#include <vector>
#include <smmintrin.h>

struct AABB
{
    constexpr AABB() noexcept
        : minimum(FLT_MAX, FLT_MAX), maximum(FLT_MIN, FLT_MIN) { }

    constexpr AABB(const Vec2& minimum, const Vec2& maximum)
        : minimum(minimum), maximum(maximum) { }

    constexpr explicit AABB(const __m128 reg)
        : reg(reg) { }

    union {
        struct { Vec2 minimum, maximum; };
        __m128 reg;
    };

    float Surface() const noexcept;
    AABB Transform(Vec2 position, Mat2 rotation) const noexcept;

    static void DrawWorld(const AABB& A) noexcept;
    static float GetSurface(const std::vector<AABB>& aabbs) noexcept;
    static AABB GetSurrounding(const std::vector<AABB>& aabbs) noexcept;
};

struct PackedAABB
{
    PackedAABB() noexcept
        : minimumX(_mm_set_ps1(FLT_MAX)), minimumY(_mm_set_ps1(FLT_MAX)),
        maximumX(_mm_set_ps1(FLT_MIN)), maximumY(_mm_set_ps1(FLT_MIN)) { }

    PackedAABB(__m128 minX, __m128 minY, __m128 maxX, __m128 maxY) noexcept
        : minimumX(minX), minimumY(minY), maximumX(maxX), maximumY(maxY) { }

    PackedAABB(const AABB& toPack) noexcept;

    __m128 minimumX;
    __m128 minimumY;
    __m128 maximumX;
    __m128 maximumY;

    static int Intersect(const PackedAABB& a, const PackedAABB& b) noexcept;
};

struct Leaf
{
    constexpr Leaf() noexcept
        : aabb(), polyIndex(0) { }
    Leaf(const AABB& aabb, size_t polyIndex) noexcept
        : aabb(aabb), polyIndex(polyIndex) { }

    AABB aabb;
    int32_t polyIndex;

    static AABB GetSurroundingAABB(const Leaf* leaves, size_t leafCount) noexcept;
    static int SortCenterX(const void* a, const void* b) noexcept;
    static int SortCenterY(const void* a, const void* b) noexcept;
};

struct ChildID
{
    constexpr ChildID() noexcept
        : index(-1), isLeaf(false) { }

    constexpr ChildID(int32_t index, bool isLeaf)
        : index(index), isLeaf(isLeaf) { }

    int32_t index : 31;
    bool isLeaf : 1;
};

struct Node2
{
    constexpr Node2() noexcept
        : childAABBs{ {}, {} }, children{ { -1, false }, { -1, false } } { }

    AABB childAABBs[2];
    ChildID children[2];
};

struct Node4
{
    Node4() noexcept
        : packedAABBs(), children{ { -1, false }, { -1, false }, { -1, false }, { -1, false } } { }

    PackedAABB packedAABBs;
    ChildID children[4];

    void SetAABB(size_t index, const AABB& aabb) noexcept;
    AABB GetAABB(size_t index) const noexcept;
};

#endif