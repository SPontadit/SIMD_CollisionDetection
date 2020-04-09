#include "SceneManager.h"

#include <iostream>
#include <string>

#include "GlobalVariables.h"
#include "PhysicEngine.h"
#include "World.h"
#include "RenderWindow.h"
#include "Renderer.h"

void CSceneManager::Reset()
{
	gVars->pPhysicEngine->Reset();

	if (gVars->pWorld != nullptr)
	{
		delete gVars->pWorld;
		gVars->pWorld = nullptr;
	}
}

void CSceneManager::AddScene(IScene* scene)
{
	m_scenes.push_back(scene);
}

void CSceneManager::LoadScene(size_t index)
{
	if (index >= m_scenes.size())
	{
		return;
	}

	Reset();

	gVars->pWorld = new CWorld();
	m_scenes[index]->Create();

	gVars->pWorld->ForEachBehavior([&](CBehaviorPtr& behavior)
	{
		behavior->Start();
	});

	m_currentScene = index;
}

void CSceneManager::ReloadScene()
{
	LoadScene(m_currentScene);
}

void CSceneManager::CheckSceneUpdate()
{
	gVars->pRenderer->DisplayText("F1: Reset scene, F2: prev scene, F3: next scene, cur scene: " + std::to_string(m_currentScene) + ", F4: debug, F5: lock FPS");

	if (gVars->pRenderWindow->JustPressedKey(Key::F2) && m_currentScene > 0)
	{
		LoadScene(m_currentScene - 1);
	}
	else if (gVars->pRenderWindow->JustPressedKey(Key::F3) && m_currentScene + 1 < m_scenes.size())
	{
		LoadScene(m_currentScene + 1);
	}
	else if (gVars->pRenderWindow->JustPressedKey(Key::F1))
	{
		ReloadScene();
	}
}