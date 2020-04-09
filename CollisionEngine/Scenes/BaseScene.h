#ifndef _BASE_SCENE_H_
#define _BASE_SCENE_H_

#include "SceneManager.h"

#include "Behavior.h"
#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "World.h"

#include "Behaviors/PolygonMoverTool.h"

class CBaseScene : public IScene
{
protected:
	CBaseScene(float borderSize = 1.0f, float worldHeight = 50.0f)
		: m_borderSize(borderSize), m_worldHeight(worldHeight){}

	virtual void Create() override
	{
		gVars->pRenderer->SetWorldHeight(m_worldHeight);
		CreateBorderRectangles();

		
		gVars->pWorld->AddBehavior<CPolygonMoverTool>(nullptr);
	}
	
	void CreateBorderRectangles()
	{
		CPolygonPtr poly;

		float halfWidth = gVars->pRenderer->GetWorldWidth() * 0.5f;
		float halfHeight = gVars->pRenderer->GetWorldHeight() * 0.5f;

		poly = gVars->pWorld->AddRectangle(halfWidth * 2.0f, m_borderSize);
		poly->position.y = -halfHeight + 0.5f * m_borderSize;
		poly->density = 0.0f;

		poly = gVars->pWorld->AddRectangle(halfWidth * 2.0f, m_borderSize);
		poly->position.y = halfHeight - 0.5f * m_borderSize;
		poly->density = 0.0f;

		poly = gVars->pWorld->AddRectangle(m_borderSize, halfHeight * 2.0f);
		poly->position.x = -halfWidth + 0.5f * m_borderSize;
		poly->density = 0.0f;

		poly = gVars->pWorld->AddRectangle(m_borderSize, halfHeight * 2.0f);
		poly->position.x = halfWidth - 0.5f * m_borderSize;
		poly->density = 0.0f;
	}

	float m_borderSize;
	float m_worldHeight;
};


#endif