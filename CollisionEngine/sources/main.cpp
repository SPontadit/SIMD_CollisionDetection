// CollisionEngine.cpp : définit le point d'entrée pour l'application console.
//
#include "stdafx.h"


#include <iostream>
#include <string>

#include "Application.h"
#include "scenes/SceneManager.h"
#include "scenes/SceneDebugCollisions.h"
#include "scenes/SceneBouncingPolys.h"
#include "scenes/SceneSpheres.h"


/*
* Entry point
*/
int _tmain(int argc, char** argv)
{
	InitApplication(1260, 768, 50.0f);

	gVars->pSceneManager->AddScene(new CSceneDebugCollisions());
	gVars->pSceneManager->AddScene(new CSceneBouncingPolys(200));
	gVars->pSceneManager->AddScene(new CSceneSpheres());


	RunApplication();
	return 0;
}

