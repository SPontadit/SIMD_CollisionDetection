#ifndef _BEHAVIOR_H_
#define _BEHAVIOR_H_

#include "Polygon.h"

class CBehavior
{
protected:
	friend class CWorld;

	CBehavior() = default;

public:
	virtual ~CBehavior() = default;

	CPolygonPtr poly;

	virtual void Start(){}
	virtual void Update(float frameTime){}

private:
	size_t	m_index = 0;
};

typedef std::shared_ptr<CBehavior>	CBehaviorPtr;

#endif