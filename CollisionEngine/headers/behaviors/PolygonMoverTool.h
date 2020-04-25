#ifndef _POLYGON_MOVER_TOOL_H_
#define _POLYGON_MOVER_TOOL_H_

#include "Behavior.h"
#include "physics/PhysicEngine.h"
#include "GlobalVariables.h"
#include "render/Renderer.h"
#include "render/RenderWindow.h"
#include "World.h"

class CPolygonMoverTool : public CBehavior
{
	size_t	GetClickedPolygon()
	{
		Vec2 pt, n;
		Vec2 mousePoint = gVars->pRenderer->ScreenToWorldPos(gVars->pRenderWindow->GetMousePos());
		size_t clickedPoly = -1;
		CPolygon poly = gVars->pWorld->polygons;

		gVars->pWorld->ForEachPolygon([&](size_t idx)
		{
			if (poly.IsPointInside(idx, mousePoint))
			{
				clickedPoly = idx;
			}
		});

		return clickedPoly;
	}

	virtual void Update(float frameTime) override
	{
		CPolygon poly = gVars->pWorld->polygons;
		if (gVars->pRenderWindow->GetMouseButton(0) || gVars->pRenderWindow->GetMouseButton(2))
		{
			if (m_selectedPolyIdx == -1)
			{
				m_selectedPolyIdx = GetClickedPolygon();
				m_prevMousePos = gVars->pRenderer->ScreenToWorldPos(gVars->pRenderWindow->GetMousePos());
				m_translate = gVars->pRenderWindow->GetMouseButton(0);
				m_clickMousePos = m_prevMousePos;

				if (m_selectedPolyIdx != -1)
					m_clickAngle = poly.rotation[m_selectedPolyIdx].GetAngle();
			}
			else
			{
				Vec2 mousePoint = gVars->pRenderer->ScreenToWorldPos(gVars->pRenderWindow->GetMousePos());

				size_t arrayIdx1 = floor(m_selectedPolyIdx / 4);
				size_t registerIdx1 = m_selectedPolyIdx % 4;

				Vec2 pos = poly.GetPosition(m_selectedPolyIdx);

				if (m_translate)
				{
					Vec2 result = pos + mousePoint - m_prevMousePos;
					poly.SetPosition(m_selectedPolyIdx, result);
					//m_selectedPoly->speed = Vec2();
				}
				else
				{
					Vec2 from = m_clickMousePos - pos;
					Vec2 to = mousePoint - pos;

					poly.rotation[m_selectedPolyIdx].SetAngle(m_clickAngle + from.Angle(to));
					//m_selectedPoly->speed = Vec2();
				}

				m_prevMousePos = mousePoint;
			}
		}
		else
		{
			//m_selectedPolyIdx.reset();
		}
	}

private:
	size_t		m_selectedPolyIdx;
	bool		m_translate;
	Vec2		m_prevMousePos;
	Vec2		m_clickMousePos;
	float		m_clickAngle;
};

#endif