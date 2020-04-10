#include "World.h"

#include "shapes/Polygon.h"

CPolygonPtr		CWorld::AddTriangle(float base, float height)
{
	CPolygonPtr poly = AddPolygon();

	poly->pointCount = 3;
	poly->pointsX = new float[3];
	poly->pointsY = new float[3];

	poly->pointsX[0] = -base * 0.5f;
	poly->pointsX[1] = base * 0.5f;
	poly->pointsX[2] = 0.0f;

	poly->pointsY[0] = -height * 0.5f;
	poly->pointsY[1] = -height * 0.5f;
	poly->pointsY[2] = height * 0.5f;

	poly->Build();

	return poly;
}

CPolygonPtr		CWorld::AddRectangle(float width, float height)
{
	CPolygonPtr poly = AddPolygon();

	poly->pointCount = 4;
	poly->pointsX = new float[4];
	poly->pointsY = new float[4];

	poly->pointsX[0] = -width * 0.5f;
	poly->pointsX[1] = width * 0.5f;
	poly->pointsX[2] = width * 0.5f;
	poly->pointsX[3] = -width * 0.5f;

	poly->pointsY[0] = -height * 0.5f;
	poly->pointsY[1] = -height * 0.5f;
	poly->pointsY[2] = height * 0.5f;
	poly->pointsY[3] = height * 0.5f;

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
	poly->pointCount = sides;
	poly->pointsX = new float[sides];
	poly->pointsY = new float[sides];

	float dAngle = 360.0f / (float)sides;
	for (size_t i = 0; i < sides; ++i)
	{
		float angle = i * dAngle;

		poly->pointsX[i] = cosf(DEG2RAD(angle));
		poly->pointsY[i] = sinf(DEG2RAD(angle));
	}
	poly->Build();

	return poly;
}

CPolygonPtr		CWorld::AddRandomPoly(const SRandomPolyParams& params)
{
	CPolygonPtr poly = AddPolygon();

	poly->pointCount = (size_t)Random(params.minPoints, params.maxPoints);
	poly->pointsX = new float[poly->pointCount];
	poly->pointsY = new float[poly->pointCount];

	float radius = Random(params.minRadius, params.maxRadius);

	float dAngle = 360.0f / (float)poly->pointCount;
	for (size_t i = 0; i < poly->pointCount; ++i)
	{
		float angle = i * dAngle + Random(-dAngle / 3.0f, dAngle / 3.0f);

		poly->pointsX[i] = cosf(DEG2RAD(angle)) * radius;
		poly->pointsY[i] = sinf(DEG2RAD(angle)) * radius;
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