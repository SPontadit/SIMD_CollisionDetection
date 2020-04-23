#ifndef _SCENE_DEBUG_COLLISIONS_H_
#define _SCENE_DEBUG_COLLISIONS_H_

#include "BaseScene.h"

#include "Behaviors/DisplayCollision.h"

class CSceneDebugCollisions : public CBaseScene
{
private:
	virtual void Create() override
	{
		CBaseScene::Create();

		CPolygonPtr firstPoly = gVars->pWorld->AddRectangle(30.0f, 20.0f); 
		firstPoly->position = Vec2(-5.0f, -5.0f);
		firstPoly->density = 0.0f;

		CPolygonPtr secondPoly = gVars->pWorld->AddRectangle(15.0f, 25.0f);
		secondPoly->position = Vec2(5.0f, 5.0f);
		secondPoly->density = 0.0f;

		/*CDisplayCollision* displayCollision = static_cast<CDisplayCollision*>(gVars->pWorld->AddBehavior<CDisplayCollision>(nullptr).get());
		displayCollision->polyA = firstPoly;
		displayCollision->polyB = secondPoly;*/
	}
};

#endif