#ifndef _SCENE_BOUNCING_POLYS_H_
#define _SCENE_BOUNCING_POLYS_H_

#include "BaseScene.h"

#include "Behaviors/SimplePolygonBounce.h"

class CSceneBouncingPolys : public CBaseScene
{
public:
	CSceneBouncingPolys(size_t polyCount)
		: m_polyCount(polyCount){}

protected:
	virtual void Create() override
	{
		CBaseScene::Create();

		gVars->pWorld->AddBehavior<CSimplePolygonBounce>(nullptr);

		float width = gVars->pRenderer->GetWorldWidth();
		float height = gVars->pRenderer->GetWorldHeight();

		SRandomPolyParams params;
		params.minRadius = 1.0f;
		params.maxRadius = 3.0f;
		params.minBounds = Vec2(-width * 0.5f + params.maxRadius * 3.0f, -height * 0.5f + params.maxRadius * 3.0f);
		params.maxBounds = params.minBounds * -1.0f;
		params.minPoints = 4;
		params.maxPoints = 4;
		params.minSpeed = 1.0f;
		params.maxSpeed = 3.0f;
		
		for (size_t i = 0; i < 20; ++i)
		{
			gVars->pWorld->AddRandomRectangle(params)->density = 0.0f;
		}
	}

private:
	size_t m_polyCount;
};

#endif