#ifndef _RENDERER_H_
#define _RENDERER_H_

#include <vector>

#include "Timer.h"
#include "Maths.h"

enum class FPS : int
{
	Unlocked = 0,
	Locked60,
	Locked30,

	Count,
};

struct SRenderText
{
	SRenderText(const std::string& _text, int _x, int _y) : text(_text), x(_x), y(_y){}

	std::string	text;
	int x, y; // screen space (0,0) left bottom corner
};

class CRenderer
{
public:
	CRenderer(float worldHeight);
	~CRenderer();

	void	SetWorldHeight(float worldHeight);
	float	GetWorldWidth() const;
	float	GetWorldHeight() const;

	void	DisplayText(const std::string& text);
	void	DisplayText(const std::string& text, int x, int y);
	void	DisplayTextWorld(const std::string& text, const Vec2& worldPos);
	void	DrawLine(const Vec2& from, const Vec2& to, float r, float g, float b);

	Vec2	ScreenToWorldPos(const Vec2& pos) const;
	Vec2	WorldToScreenPos(const Vec2& pos) const;

	void	Init();
	void	Reset();
	void	Reshape(int width, int height);
	void	Update();

private:
	void	SetProjectionMatrix();
	void	PreRenderFrame();
	void	DrawFPS(float frameTime);
	void	UpdateWorld(float frameTime);
	void	RenderPolygons();
	void	RenderTexts();
	void	UpdateLockFPS();

	float	UpdateFrameTime();

private:
	float m_worldHeight; // height in world units

	CTimer m_frameTimer;

	std::vector<SRenderText>	m_renderTexts;
	int							m_textCursor;

	struct dtx_font* m_font;

	float	m_lastFPS;
	float	m_lastFPSSince;
	FPS		m_FPS;
};

#endif