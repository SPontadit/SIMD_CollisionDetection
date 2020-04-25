#ifndef _WORLD_H_
#define _WORLD_H_

#include <vector>

#include "shapes/Polygon.h"
#include "shapes/AABB.h"
#include "behaviors/Behavior.h"

struct SRandomPolyParams
{
	size_t	minPoints, maxPoints;
	float	minRadius, maxRadius;
	Vec2	minBounds, maxBounds;
	float	minSpeed, maxSpeed;
};

class CWorld
{
public:
	CWorld()
		: polygons(CPolygon())
	{

	}

	size_t			AddRectangle(float width, float height, const Vec2& position);
	size_t			AddRandomRectangle(const SRandomPolyParams& params);

	size_t			AddPolygon();
	void			RemovePolygon(size_t index);

	// WARNING
	template<class TBehavior>
	CBehaviorPtr	AddBehavior(CPolygonPtr poly)
	{
		CBehaviorPtr behavior(new TBehavior());
		behavior->m_index = m_behaviors.size();
		behavior->polyIdx = 0;
		m_behaviors.push_back(behavior);

		return behavior;
	}
	void			RemoveBehavior(CBehaviorPtr behavior);

	template<typename TFunctor>
	void	ForEachPolygon(TFunctor functor)
	{
		//for (CPolygonPtr poly : m_polygons)
		for (size_t i = 0; i < polygons.polyCount; ++i)
		{
			functor(i);
		}
	}
	size_t		GetPolygonCount() const;
	CPolygonPtr	GetPolygon(size_t index);

	template<typename TFunctor>
	void	ForEachBehavior(TFunctor functor)
	{
		for (CBehaviorPtr behavior : m_behaviors)
		{
			functor(behavior);
		}
	}

	void Update(float frameTime);
	void RenderPolygons();

	CPolygon polygons;
protected:
	//std::vector<CPolygonPtr>	m_polygons;
	std::vector<CBehaviorPtr>	m_behaviors;
};

#endif