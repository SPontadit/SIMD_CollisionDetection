#include "World.h"

#include "Polygon.h"

CPolygonPtr		CWorld::AddTriangle(float base, float height)
{
	CPolygonPtr poly = AddPolygon();
	poly->points.push_back({ -base * 0.5f, -height * 0.5f });
	poly->points.push_back({ base * 0.5f, -height * 0.5f });
	poly->points.push_back({ 0.0f, height * 0.5f });
	poly->Build();

	return poly;
}

CPolygonPtr		CWorld::AddRectangle(float width, float height)
{
	CPolygonPtr poly = AddPolygon();
	poly->points.push_back({ -width * 0.5f, -height * 0.5f });
	poly->points.push_back({ width * 0.5f, -height * 0.5f });
	poly->points.push_back({ width * 0.5f, height * 0.5f });
	poly->points.push_back({ -width * 0.5f, height * 0.5f });
	poly->Build();

	return poly;
}

CPolygonPtr		CWorld::AddSquare(float size)
{
	return AddRectangle(size, size);
}

CPolygonPtr		CWorld::AddSymetricPolygon(float radius, size_t sides)
{
	CPolygonPtr poly = AddPolygon();
	float dAngle = 360.0f / (float)sides;
	for (size_t i = 0; i < sides; ++i)
	{
		float angle = i * dAngle;

		Vec2 point = Vec2(cosf(DEG2RAD(angle)), sinf(DEG2RAD(angle))) * radius;
		poly->points.push_back(point);
	}
	poly->Build();

	return poly;
}

CPolygonPtr		CWorld::AddRandomPoly(const SRandomPolyParams& params)
{
	size_t pointsCount = (size_t)Random(params.minPoints, params.maxPoints);
	float radius = Random(params.minRadius, params.maxRadius);

	CPolygonPtr poly = AddPolygon();
	float dAngle = 360.0f / (float)pointsCount;
	for (size_t i = 0; i < pointsCount; ++i)
	{
		float angle = i * dAngle + Random(-dAngle / 3.0f, dAngle / 3.0f);
		float dist = radius;

		Vec2 point = Vec2(cosf(DEG2RAD(angle)), sinf(DEG2RAD(angle))) * dist;
		poly->points.push_back(point);
	}

	poly->Build();
	poly->rotation.SetAngle(Random(-180.0f, 180.0f));
	poly->position.x = Random(params.minBounds.x, params.maxBounds.x);
	poly->position.y = Random(params.minBounds.y, params.maxBounds.y);

	Mat2 rot;
	rot.SetAngle(Random(-180.0f, 180.0f));
	poly->speed = rot.X * Random(params.minSpeed, params.maxSpeed);

	return poly;
}

CPolygonPtr		CWorld::AddPolygon()
{
	CPolygonPtr poly( new CPolygon(m_polygons.size()) );
	m_polygons.push_back(poly);
	return poly;
}

void	CWorld::RemovePolygon(CPolygonPtr poly)
{
	size_t index = poly->m_index;

	if (index + 1 < m_polygons.size())
	{
		CPolygonPtr movedPoly = m_polygons[m_polygons.size() - 1];
		m_polygons[index] = movedPoly;
		movedPoly->m_index = index;
	}
}

void	CWorld::RemoveBehavior(CBehaviorPtr behavior)
{
	if (behavior->poly)
	{
		RemovePolygon(behavior->poly);
	}

	size_t index = behavior->m_index;

	if (index + 1 < m_polygons.size())
	{
		CBehaviorPtr movedBhv = m_behaviors[m_behaviors.size() - 1];
		m_behaviors[index] = movedBhv;
		movedBhv->m_index = index;
	}
}

size_t	CWorld::GetPolygonCount() const
{
	return m_polygons.size();
}

CPolygonPtr		CWorld::GetPolygon(size_t index)
{
	return m_polygons[index];
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
	for (CPolygonPtr polygon : m_polygons)
	{
		polygon->Draw();
	}
}