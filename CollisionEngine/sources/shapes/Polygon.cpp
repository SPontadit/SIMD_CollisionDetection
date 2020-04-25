#include "shapes/Polygon.h"

#include <GL/glu.h>

#include "physics/PhysicEngine.h"
#include "shapes/AABB.h"

CPolygon::CPolygon()
{
	__m128 zero = _mm_set_ps1(0.0f);
	for (size_t i = 0; i < polyCount; i++)
	{
		registerRotation[i] = zero;
		positionX[i] = zero;
		positionY[i] = zero;
		halfExtentX[i] = zero;
		halfExtentY[i] = zero;
		density[i] = 0.0f;
		speed[i] = Vec2();
		m_vertexBufferId[i] = 0;
	}
}

CPolygon::~CPolygon()
{
}

void CPolygon::Destroy()
{
	for (size_t i = 0; i < polyCount; i++)
		DestroyBuffers(i);
}

void CPolygon::Build(const size_t polyIdx, float pointsX[4], float pointsY[4])
{
	m_lines[polyIdx].clear();

	CreateBuffers(polyIdx, pointsX, pointsY);
	BuildLines(polyIdx, pointsX, pointsY);
}

void CPolygon::Draw(const size_t index)
{
	Vec2 position = GetPosition(index);

	// Set transforms (qssuming model view mode is set)
	float transfMat[16] = {	rotation[index].X.x, rotation[index].X.y, 0.0f, 0.0f,
							rotation[index].Y.x, rotation[index].Y.y, 0.0f, 0.0f,
							0.0f, 0.0f, 0.0f, 1.0f,
							position.x, position.y, -1.0f, 1.0f };
	glPushMatrix();
	glMultMatrixf(transfMat);

	// Draw vertices
	BindBuffers(index);
	glDrawArrays(GL_LINE_LOOP, 0, 4);
	
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();
}

Vec2 CPolygon::GetPosition(const size_t index) const
{
	size_t arrayIdx = floor(index / 4);
	size_t registerIdx = index % 4;

	return { positionX[arrayIdx].m128_f32[registerIdx], 
		positionY[arrayIdx].m128_f32[registerIdx] };
}

void CPolygon::SetPosition(const size_t index, const Vec2& position)
{
	size_t arrayIdx = floor(index / 4);
	size_t registerIdx = index % 4;

	positionX[arrayIdx].m128_f32[registerIdx] = position.x;
	positionY[arrayIdx].m128_f32[registerIdx] = position.y;
}

void CPolygon::SetExtent(const size_t index, const Vec2& halfExtent)
{
	size_t arrayIdx = floor(index / 4);
	size_t registerIdx = index % 4;

	halfExtentX[arrayIdx].m128_f32[registerIdx] = halfExtent.x;
	halfExtentY[arrayIdx].m128_f32[registerIdx] = halfExtent.y;
}

Vec2	CPolygon::TransformPoint(const size_t index, const Vec2& point) const
{
	return GetPosition(index) + rotation[index] * point;
}

Vec2	CPolygon::InverseTransformPoint(const size_t index, const Vec2& point) const
{
	return rotation[index].GetInverse() * (point - GetPosition(index));
}

bool	CPolygon::IsPointInside(const size_t index, const Vec2& point) const
{
	float maxDist = -FLT_MAX;

	for (const Line& line : m_lines[index])
	{
		Vec2 pos = GetPosition(index);
		Line globalLine = line.Transform(rotation[index], pos);
		float pointDist = globalLine.GetPointDist(point);
		maxDist = Max(maxDist, pointDist);
	}

	return maxDist <= 0.0f;
}

bool	CPolygon::CheckCollision(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist) const
{
	return false;
}

void CPolygon::CreateBuffers(const size_t polyIdx, float pointsX[4], float pointsY[4])
{
	DestroyBuffers(polyIdx);

	constexpr size_t pointCount = 4;

	float* vertices = new float[3 * pointCount];
	for (size_t i = 0; i < pointCount; ++i)
	{
		vertices[3 * i] = pointsX[i];
		vertices[3 * i + 1] = pointsY[i];
		vertices[3 * i + 2] = 0.0f;
	}

	GLuint id;
	glGenBuffers(1, &id);
	glBindBuffer(GL_ARRAY_BUFFER, id);

	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * pointCount, vertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	m_vertexBufferId[polyIdx] = id;

	delete[] vertices;
}

void CPolygon::BindBuffers(const size_t polyIdx)
{
	if (m_vertexBufferId[polyIdx] != 0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId[polyIdx]);

		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, (void*)0);
	}
}

void CPolygon::DestroyBuffers(const size_t polyIdx)
{
	if (m_vertexBufferId[polyIdx] != 0)
	{
		glDeleteBuffers(1, &m_vertexBufferId[polyIdx]);
		m_vertexBufferId[polyIdx] = 0;
	}
}

void CPolygon::BuildLines(const size_t polyIdx, float pointsX[4], float pointsY[4])
{
	constexpr size_t pointCount = 4;

	for (size_t index = 0; index < pointCount; ++index)
	{
		int next = (index + 1) % pointCount;

		Vec2 pointA(pointsX[index], pointsY[index]);
		Vec2 pointB(pointsX[next], pointsY[next]);

		Vec2 lineDir = (pointA - pointB).Normalized();

		m_lines[polyIdx].push_back(Line(pointB, lineDir));
	}
}