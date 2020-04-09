#include "Timer.h"

void CTimer::Start()
{
	QueryPerformanceFrequency(&m_frequency);
	QueryPerformanceCounter(&m_startTime);
}

void CTimer::Stop()
{
	QueryPerformanceCounter(&m_stopTime);
}

float	CTimer::GetDuration() const
{
	return (float)(m_stopTime.QuadPart - m_startTime.QuadPart) / (float)m_frequency.QuadPart;
}