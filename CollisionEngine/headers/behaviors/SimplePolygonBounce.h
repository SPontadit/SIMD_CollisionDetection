#ifndef _SIMPLE_COLLISION_BOUNCE_H_
#define _SIMPLE_COLLISION_BOUNCE_H_

#include "Behavior.h"
#include "physics/PhysicEngine.h"
#include "GlobalVariables.h"
#include "render/Renderer.h"
#include "World.h"

class CSimplePolygonBounce : public CBehavior
{
private:
	virtual void Update(float frameTime) override
	{
		//gVars->pPhysicEngine->ForEachCollision([&](const SCollision& collision)
		//{
		//	collision.polyA->position += collision.normal * collision.distance * -0.5f;
		//	collision.polyB->position += collision.normal * collision.distance * 0.5f;

		//	collision.polyA->speed.Reflect(collision.normal);
		//	collision.polyB->speed.Reflect(collision.normal);
		//});

		float hWidth = gVars->pRenderer->GetWorldWidth() * 0.5f;
		float hHeight = gVars->pRenderer->GetWorldHeight() * 0.5f;

		CPolygon polygons = gVars->pWorld->polygons;

		gVars->pWorld->ForEachPolygon([&](size_t idx)
		{
				Vec2 position = polygons.GetPosition(idx);
				Vec2 speed = polygons.speed[idx];

			position += speed * frameTime;
			polygons.SetPosition(idx, position);

			if (position.x < -hWidth)
			{
				position.x = -hWidth;
				speed.x *= -1.0f;
			}
			else if (position.x > hWidth)
			{
				position.x = hWidth;
				speed.x *= -1.0f;
			}

			if (position.y < -hHeight)
			{
				position.y = -hHeight;
				speed.y *= -1.0f;
			}
			else if (position.y > hHeight)
			{
				position.y = hHeight;
				speed.y *= -1.0f;
			}

			polygons.SetPosition(idx, position);
			polygons.speed[idx] = speed;
		});
	}
};

#endif