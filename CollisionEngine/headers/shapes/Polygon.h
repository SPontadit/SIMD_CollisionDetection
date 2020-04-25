#ifndef _POLYGON_H_
#define _POLYGON_H_

#include <GL/glew.h>
#include <vector>
#include <memory>
#include <immintrin.h>

#include "Maths.h"

#define MAX_POLY 100

class CPolygon
{
private:
	friend class CWorld;

	CPolygon();
public:
	~CPolygon();

	//Vec2				position;
	//Mat2				rotation;
	////std::vector<Vec2>	points;
	//float*				pointsX;
	//float*				pointsY;
	//size_t				pointCount;
	//Vec2				halfExtent;
	size_t				polyCount;

	union
	{
		__m128				registerRotation[MAX_POLY * 4];
		Mat2				rotation[MAX_POLY * 4];
	};

	__m128				positionX[MAX_POLY];
	__m128				positionY[MAX_POLY];
	__m128				halfExtentX[MAX_POLY];
	__m128				halfExtentY[MAX_POLY];

	void				Build(const size_t polyIdx, float pointsX[4], float pointsY[4]);
	void				Draw(const size_t index);
	//size_t				GetIndex() const;
	Vec2				GetPosition(const size_t index) const;
	void				SetExtent(const size_t index, const Vec2& halfExtent);
	void				SetPosition(const size_t index, const Vec2& position);

	float				GetArea() const;

	Vec2				TransformPoint(const size_t index, const Vec2& point) const;
	Vec2				InverseTransformPoint(const size_t index, const Vec2& point) const;

	// if point is outside then returned distance is negative (and doesn't make sense)
	bool				IsPointInside(const size_t index, const Vec2& point) const;

	bool				CheckCollision(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist) const;
	void				Destroy();

	// Physics
	float				density;
	Vec2				speed[MAX_POLY*4];

private:
	void				CreateBuffers(const size_t polyIdx, float pointsX[4], float pointsY[4]);
	void				BindBuffers(const size_t polyIdx);
	void				DestroyBuffers(const size_t polyIdx);

	void				BuildLines(const size_t polyIdx, float pointsX[4], float pointsY[4]);

	GLuint				m_vertexBufferId[MAX_POLY*4];
	size_t				m_index;

	std::vector<Line>	m_lines[MAX_POLY*4];
};

typedef std::shared_ptr<CPolygon>	CPolygonPtr;

#endif