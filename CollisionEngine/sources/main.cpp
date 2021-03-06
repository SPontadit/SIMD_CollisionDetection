// CollisionEngine.cpp�: d�finit le point d'entr�e pour l'application console.
//
#include "stdafx.h"


#include <iostream>
#include <string>

#include "Application.h"
#include "scenes/SceneManager.h"
#include "scenes/SceneDebugCollisions.h"
#include "scenes/SceneBouncingPolys.h"


/*
* Entry point
*/
int _tmain(int argc, char** argv)
{
	InitApplication(1260, 768, 50.0f);

	gVars->pSceneManager->AddScene(new CSceneDebugCollisions());
	gVars->pSceneManager->AddScene(new CSceneBouncingPolys(200));


	RunApplication();
	return 0;
}

