#ifndef _TIMER_H_
#define _TIMER_H_

#include <Windows.h>

class CTimer
{
public:
	void Start();
	void Stop();

	float	GetDuration() const;

private:
	LARGE_INTEGER	m_frequency;
	LARGE_INTEGER	m_startTime;
	LARGE_INTEGER	m_stopTime;
};

#endif