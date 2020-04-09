#ifndef _RENDER_WINDOW_SDL_H_
#define _RENDER_WINDOW_SDL_H_

#include "RenderWindow.h"

#include <unordered_map>

class CSDLRenderWindow : public CRenderWindow
{
public:
	CSDLRenderWindow(int width, int height);

	virtual void	Init() override;
	virtual void	Reshape(int width, int height) override;

	virtual Vec2	GetMousePos() override;
	virtual bool	GetMouseButton(int button) override;
	virtual bool	IsPressingKey(Key key) override;
	virtual bool	JustPressedKey(Key key) override;

private:
	bool			ProcessEvents();
	void			ResetKeys();

	std::unordered_map<unsigned int, Key>	m_sdlKeyMap;
	bool									m_pressedKeys[(size_t)Key::Count];
	bool									m_justPressedKeys[(size_t)Key::Count];
};

#endif