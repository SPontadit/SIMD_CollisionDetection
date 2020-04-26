#ifndef _GLOBAL_VARIABLES_H_
#define _GLOBAL_VARIABLES_H_

struct SGlobalVariables
{
	class CRenderWindow*	pRenderWindow;
	class CRenderer*		pRenderer;
	class CWorld*			pWorld;
	class CSceneManager*	pSceneManager;
	class CPhysicEngine*	pPhysicEngine;

	bool					bDebug;
};

extern SGlobalVariables*	gVars;

#endif