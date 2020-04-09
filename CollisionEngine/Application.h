#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include "GlobalVariables.h"
#include "SDLRenderWindow.h"
#include "PhysicEngine.h"
#include "Renderer.h"
#include "SceneManager.h"
#include "World.h"

void InitApplication(int width, int height, float worldHeight)
{
	gVars = new SGlobalVariables();

	gVars->pRenderWindow = new CSDLRenderWindow(width, height);
	gVars->pRenderer = new CRenderer(worldHeight);
	gVars->pSceneManager = new CSceneManager();
	gVars->pPhysicEngine = new CPhysicEngine();

	gVars->bDebug = false;
}

void RunApplication()
{
	gVars->pRenderWindow->Init();
}

#endif