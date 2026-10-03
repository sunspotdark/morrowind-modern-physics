#ifndef OPENMW_MWPHYSICS_COLLISIONTYPE_H
#define OPENMW_MWPHYSICS_COLLISIONTYPE_H

namespace MWPhysics
{

    enum CollisionType
    {
        CollisionType_World = 1 << 0,
        CollisionType_Door = 1 << 1,
        CollisionType_Actor = 1 << 2,
        CollisionType_HeightMap = 1 << 3,
        CollisionType_Projectile = 1 << 4,
        CollisionType_Water = 1 << 5,
        CollisionType_Default
        = CollisionType_World | CollisionType_HeightMap | CollisionType_Actor | CollisionType_Door,
        CollisionType_AnyPhysical = CollisionType_World | CollisionType_HeightMap | CollisionType_Actor
            | CollisionType_Door | CollisionType_Projectile | CollisionType_Water,
        CollisionType_CameraOnly = 1 << 6,
        CollisionType_VisualOnly = 1 << 7,
        // Simulated rigid bodies (knockable clutter). Deliberately not part of CollisionType_Default or
        // CollisionType_AnyPhysical so existing queries and actor movement ignore them.
        CollisionType_Dynamic = 1 << 8,
        // Detailed copies of furniture collision (from the rendered mesh), for simulated objects only. The
        // regular, often simplified, collision of the same object then ignores simulated objects.
        CollisionType_DynamicDetail = 1 << 9,
        // Everything static that simulated objects rest on and collide with.
        CollisionType_DynamicSupport
        = CollisionType_World | CollisionType_Door | CollisionType_HeightMap | CollisionType_DynamicDetail
    };

}

#endif
