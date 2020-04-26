==== COLLISION DETECTION ======================================================
==== CODE
Interesting code in the following places:

 - PhysicEngine.cpp -> CPhysicEngine::Step
 - AABB.h
 - AABB.cpp

++++ BVH CONSTRUCTION
 - PhysicEngine.cpp -> CPhysicEngine::BuildAABBTree
                    -> CPhysicEngine::BVH2Recurse
                    -> CPhysicEngine::BVH2ToBVH4

++++ BROAD PHASE
 - BroadPhaseAABBTree.cpp

++++ NARROW PHASE
 - PhysicEngine.cpp -> CPhysicEngine::SIMD_Shuffle_OBBCollisionTest
 
==== CONTROLS
F4 to show debug info, also toggles BVH4 display
Polygons can be moved with left click and rotated with right click

==== RAY TRACING ==============================================================
Builds in directory RAY_TRACING/Out/x64