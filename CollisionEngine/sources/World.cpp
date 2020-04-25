#include "World.h"

#include "GlobalVariables.h"
#include "physics/PhysicEngine.h"

size_t		CWorld::AddRectangle(float width, float height, const Vec2& position)
{
	size_t polyIdx = AddPolygon();

	const float halfWidth  = fabs(width) * 0.5f;
	const float halfHeight = fabs(height) * 0.5f;

	float pointsX[4];
	float pointsY[4];

	pointsX[0] = -halfWidth;
	pointsX[1] = halfWidth;
	pointsX[2] = halfWidth;
	pointsX[3] = -halfWidth;

	pointsY[0] = -halfHeight;
	pointsY[1] = -halfHeight;
	pointsY[2] = halfHeight;
	pointsY[3] = halfHeight;

	polygons.Build(polyIdx, pointsX, pointsY);

	polygons.SetExtent(polyIdx, { halfWidth, halfHeight });

	gVars->pPhysicEngine->AddLocalAABB(AABB({ -halfWidth, -halfHeight}, {halfWidth, halfHeight}));

	polygons.SetPosition(polyIdx, position);

	return polyIdx;
}

size_t		CWorld::AddRandomRectangle(const SRandomPolyParams& params)
{
	size_t polyIdx = AddPolygon();

	const float halfWidth = fabs(Random(params.minRadius, params.maxRadius)) * 0.5f;
	const float halfHeight = fabs(Random(params.minRadius, params.maxRadius)) * 0.5f;

	float pointsX[4];
	float pointsY[4];

	pointsX[0] = -halfWidth;
	pointsX[1] = halfWidth;
	pointsX[2] = halfWidth;
	pointsX[3] = -halfWidth;

	pointsY[0] = -halfHeight;
	pointsY[1] = -halfHeight;
	pointsY[2] = halfHeight;
	pointsY[3] = halfHeight;

	polygons.Build(polyIdx, pointsX, pointsY);

	polygons.SetExtent(polyIdx, { halfWidth, halfHeight });

	polygons.rotation[polyIdx].SetAngle(Random(-180.0f, 180.0f));

	polygons.SetPosition(polyIdx, { Random(params.minBounds.x, params.maxBounds.x), Random(params.minBounds.y, params.maxBounds.y) });

	Mat2 rot;
	rot.SetAngle(Random(-180.0f, 180.0f));
	polygons.speed[polyIdx] = rot.X * Random(params.minSpeed, params.maxSpeed);

	gVars->pPhysicEngine->AddLocalAABB(AABB({ -halfWidth, -halfHeight }, { halfWidth, halfHeight }));

	return polyIdx;
}

size_t		CWorld::AddPolygon()
{
	//CPolygonPtr poly( new CPolygon(m_polygons.size()) );
	//m_polygons.push_back(poly);
	size_t idx = polygons.polyCount;
	polygons.polyCount++;
	return idx;
}

// WARNING
void	CWorld::RemovePolygon(size_t index)
{
	polygons.polyCount--;
	//if (index + 1 < m_polygons.size())
	//{
	//	CPolygonPtr movedPoly = m_polygons[m_polygons.size() - 1];
	//	m_polygons[index] = movedPoly;
	//	movedPoly->m_index = index;
	//	m_polygons.pop_back();

	//	gVars->pPhysicEngine->RemoveLocalAABB(index);
	//}
}

// WARNING
void	CWorld::RemoveBehavior(CBehaviorPtr behavior)
{
	//if (behavior->poly)
	//{
	//	RemovePolygon(behavior->poly);
	//}

	//size_t index = behavior->m_index;

	//if (index + 1 < m_polygons.size())
	//{
	//	CBehaviorPtr movedBhv = m_behaviors[m_behaviors.size() - 1];
	//	m_behaviors[index] = movedBhv;
	//	movedBhv->m_index = index;
	//}
}

size_t	CWorld::GetPolygonCount() const
{
	return polygons.polyCount;
	//return m_polygons.size();
}

void	CWorld::Update(float frameTime)
{
	for(CBehaviorPtr behavior : m_behaviors)
	{
		behavior->Update(frameTime);
	}
}

void	CWorld::RenderPolygons()
{
	for (size_t i = 0; i < polygons.polyCount; ++i)
		polygons.Draw(i);
}