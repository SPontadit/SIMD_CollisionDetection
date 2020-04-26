#ifndef _RENDER_WINDOW_H_
#define _RENDER_WINDOW_H_

#include "Maths.h"

enum class Key : unsigned int
{
	F1 = 0,
	F2,
	F3,
	F4,
	F5,

	Count,
};

class CRenderWindow
{
public:
	CRenderWindow(int width, int height)
		: m_width(width), m_height(height){}

	int				GetWidth() const	{ return m_width;}
	int				Getheight() const	{ return m_height; }

	virtual void	Init() = 0;
	virtual void	Reshape(int width, int height){}

	virtual Vec2	GetMousePos() = 0;
	virtual bool	GetMouseButton(int button) = 0;
	virtual bool	IsPressingKey(Key key) = 0;
	virtual bool	JustPressedKey(Key key) = 0;

protected:
	int	m_width;
	int m_height;
};

#endif