#include <stdlib.h>
#include <GL/glew.h>

#include <stdio.h>
#include <iostream>
#include <string>

#include "GlobalVariables.h"
#include "Renderer.h"
#include "RenderWindow.h"
#include "Polygon.h"
#include "PhysicEngine.h"
#include "SceneManager.h"
#include "World.h"

#include "drawtext.h"

CRenderer::CRenderer(float worldHeight)
	: m_worldHeight(worldHeight), m_lastFPS(0.0f), m_lastFPSSince(0.0f), m_textCursor(0), m_FPS(FPS::Unlocked)
{}

CRenderer::~CRenderer(){}

void CRenderer::SetWorldHeight(float worldHeight)
{
	m_worldHeight = worldHeight;
}

float	CRenderer::GetWorldWidth() const
{
	int width = gVars->pRenderWindow->GetWidth();
	int height = gVars->pRenderWindow->Getheight();
	float ratio = (float)width / (float)height;
	return ratio * m_worldHeight;
}

float	CRenderer::GetWorldHeight() const
{
	return m_worldHeight;
}

void CRenderer::DisplayText(const std::string& text)
{
	DisplayText(text, 50, gVars->pRenderWindow->Getheight() - 50 - 30 * m_textCursor++);
}

void CRenderer::DisplayText(const std::string& text, int x, int y)
{
	m_renderTexts.push_back(SRenderText(text, x, y));
}

void CRenderer::DisplayTextWorld(const std::string& text, const Vec2& worldPos)
{
	Vec2 screenPos = WorldToScreenPos(worldPos);
	DisplayText(text, (int)screenPos.x, (int)screenPos.y);
}


void CRenderer::DrawLine(const Vec2& from, const Vec2& to, float r, float g, float b)
{
	glColor3f(r, g, b);
	glBegin(GL_LINES);
	glVertex3f(from.x, from.y, -1.0f);
	glVertex3f(to.x, to.y, -1.0f);
	glEnd();
}

Vec2 CRenderer::ScreenToWorldPos(const Vec2& pos) const
{
	float width = (float)gVars->pRenderWindow->GetWidth();
	float height = (float)gVars->pRenderWindow->Getheight();

	return (pos - Vec2(width, height) * 0.5f) * (m_worldHeight / height);
}

Vec2 CRenderer::WorldToScreenPos(const Vec2& pos) const
{
	float width = (float)gVars->pRenderWindow->GetWidth();
	float height = (float)gVars->pRenderWindow->Getheight();

	return pos * (height / m_worldHeight) + Vec2(width, height) * 0.5f;
}

void CRenderer::Init()
{
	// Init font
	m_font = dtx_open_font("font.ttf", 24);
	dtx_use_font(m_font, 24);

	m_frameTimer.Start();

	// Load scene 0
	gVars->pSceneManager->LoadScene(0);
}

void CRenderer::Reset()
{
	gVars->pSceneManager->Reset();
}

void CRenderer::Reshape(int width, int height)
{
	glViewport(0, 0, width, height);
}

void CRenderer::Update()
{
	CTimer timer;

	if (gVars->pRenderWindow->JustPressedKey(Key::F4))
	{
		gVars->bDebug = !gVars->bDebug;
	}

	gVars->pSceneManager->CheckSceneUpdate();

	PreRenderFrame();

	float frameTime = UpdateFrameTime();
	DrawFPS(frameTime);

	gVars->pPhysicEngine->Step(frameTime);
	
	timer.Start();
	UpdateWorld(frameTime);
	timer.Stop(); 
	if (gVars->bDebug)
	{
		DisplayText("Update duration : " + std::to_string(timer.GetDuration()));
	}

	timer.Start();
	RenderPolygons();
	timer.Stop();
	if (gVars->bDebug)
	{
		DisplayText("Render duration : " + std::to_string(timer.GetDuration()));
	}

	RenderTexts();

	UpdateLockFPS();
}

void  CRenderer::SetProjectionMatrix()
{
	int width = gVars->pRenderWindow->GetWidth();
	int height = gVars->pRenderWindow->Getheight();
	float ratio = (float)width / (float)height;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-m_worldHeight * 0.5f * ratio, m_worldHeight * 0.5f * ratio, -m_worldHeight * 0.5f, m_worldHeight * 0.5f, 0.1f, 10.0f);
}

void  CRenderer::PreRenderFrame()
{
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	SetProjectionMatrix();

	glMatrixMode(GL_MODELVIEW);
	glTranslatef(0.0f, 0.0f, 0.0f); // move camera here
}

void  CRenderer::DrawFPS(float frameTime)
{
	if (m_lastFPSSince <= 0.0f)
	{
		m_lastFPS = 1.0f / frameTime;
		m_lastFPSSince = 0.5f;
	}
	else
	{
		m_lastFPSSince -= frameTime;
	}

	int width = gVars->pRenderWindow->GetWidth();
	int height = gVars->pRenderWindow->Getheight();
	DisplayText(std::string("FPS : ") + std::to_string(m_lastFPS), width - 200, height - 30);
}

void  CRenderer::UpdateWorld(float frameTime)
{
	if (gVars->pWorld)
	{
		gVars->pWorld->Update(frameTime);
	}
}

void  CRenderer::RenderPolygons()
{
	glColor3f(0.0f, 0.0f, 0.0f);

	glPushMatrix();

	if (gVars->pWorld)
	{
		gVars->pWorld->RenderPolygons();
	}

	glPopMatrix();
}

void  CRenderer::RenderTexts()
{
	int width = gVars->pRenderWindow->GetWidth();
	int height = gVars->pRenderWindow->Getheight();

	// Black text
	glColor3f(0.0f, 0.0f, 0.0f);

	// Set proj matrix to screen space
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, width, 0, height, -1, 1);

	// Reset model matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	for (const SRenderText& text : m_renderTexts)
	{
		glPushMatrix();

		glTranslatef((float)text.x, (float)text.y, 0.0f);
		dtx_string(text.text.c_str());

		glPopMatrix();
	}

	m_renderTexts.clear();
	m_textCursor = 0;
}

void  CRenderer::UpdateLockFPS()
{
	if (gVars->pRenderWindow->JustPressedKey(Key::F5))
	{
		m_FPS = (FPS)(((int)m_FPS + 1) % (int)FPS::Count);
	}

	if (m_FPS != FPS::Unlocked)
	{
		float frameTimeLimit = (m_FPS == FPS::Locked30) ? 1.0f / 30.0f : 1.0f / 60.0f;
		do
		{
			m_frameTimer.Stop();
		} while (m_frameTimer.GetDuration() < frameTimeLimit);
	}
}

float  CRenderer::UpdateFrameTime()
{
	m_frameTimer.Stop();
	float frameTime = m_frameTimer.GetDuration();
	m_frameTimer.Start();

	return frameTime;
}