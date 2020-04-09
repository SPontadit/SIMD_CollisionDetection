#ifndef _SCENE_SPHERES_H_
#define _SCENE_SPHERES_H_

#include "BaseScene.h"

#include "Behaviors/SphereSimulation.h"

class CSceneSpheres : public IScene
{
private:
	virtual void Create() override
	{
		gVars->pWorld->AddBehavior<CPolygonMoverTool>(nullptr);
		gVars->pWorld->AddBehavior<CSphereSimulation>(nullptr);
	}
};

#endif