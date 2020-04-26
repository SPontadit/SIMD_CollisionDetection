#ifndef _SCENE_MANAGER_H_
#define _SCENE_MANAGER_H_

#include <vector>

class IScene
{
public:
	virtual void	Create() = 0;
};

class CSceneManager
{
public:
	void Reset();

	void AddScene(IScene* scene);
	void LoadScene(size_t index);
	void ReloadScene();

	void CheckSceneUpdate();

private:
	std::vector<IScene*>	m_scenes;
	size_t					m_currentScene = 0;
};

#endif