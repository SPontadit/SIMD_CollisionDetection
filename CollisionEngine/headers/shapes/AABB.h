#ifndef _AABB_H_
#define _AABB_H_

#include <cfloat>
#include "Maths.h"
#include <vector>
#include <smmintrin.h>

struct PackedAABB
{
    PackedAABB(__m128 minX, __m128 minY, __m128 maxX, __m128 maxY)
        : minimumX(minX), minimumY(minY), maximumX(maxX), maximumY(maxY) { }

    __m128 minimumX;
    __m128 minimumY;
    __m128 maximumX;
    __m128 maximumY;
};


struct AABB
{
    constexpr AABB()
        : minimum(FLT_MAX, FLT_MAX), maximum(FLT_MIN, FLT_MIN) { }

    constexpr AABB(const Vec2& minimum, const Vec2& maximum)
        : minimum(minimum), maximum(maximum) { }

    constexpr explicit AABB(const __m128 reg)
        : reg(reg) { }

    union {
        struct { Vec2 minimum, maximum; };
        __m128 reg;
    };

    AABB Transform(Vec2 position, Mat2 rotation) const noexcept;

    static void DrawWorld(const AABB& A) noexcept;
    static float GetSurface(const std::vector<AABB>& aabbs) noexcept;
    static AABB GetSurrounding(const std::vector<AABB>& aabbs) noexcept;
    static int Intersect(const PackedAABB test, const PackedAABB node) noexcept;
};


struct Node
{
    Node() = default;
    
    Node(const std::vector<AABB>& nodes) noexcept;

    bool isLeaf;
    Node* leftNode;
    Node* rightNode;
    AABB aabb;
};

#endif