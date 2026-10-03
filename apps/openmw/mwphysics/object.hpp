#ifndef OPENMW_MWPHYSICS_OBJECT_H
#define OPENMW_MWPHYSICS_OBJECT_H

#include "ptrholder.hpp"
#include <memory>

#include <LinearMath/btTransform.h>
#include <osg/Node>

#include <map>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

class btCollisionShape;
class btRigidBody;

namespace Resource
{
    class BulletShapeInstance;
}

namespace MWPhysics
{
    class PhysicsTaskScheduler;
    class DynamicMotionState;

    enum ScriptedCollisionType : char
    {
        ScriptedCollisionType_None = 0,
        ScriptedCollisionType_Actor = 1,
        // Note that this isn't 3, colliding with a player doesn't count as colliding with an actor
        ScriptedCollisionType_Player = 2
    };

    class Object final : public PtrHolder
    {
    public:
        Object(const MWWorld::Ptr& ptr, std::shared_ptr<Resource::BulletShapeInstance> shapeInstance,
            osg::Quat rotation, int collisionType, PhysicsTaskScheduler* scheduler);
        /// Construct a simulated rigid body with the given mass. Its shape is the convex hull of hullPoints (in
        /// unscaled mesh space), or a box around the mesh if there are none.
        Object(const MWWorld::Ptr& ptr, std::shared_ptr<Resource::BulletShapeInstance> shapeInstance,
            osg::Quat rotation, float mass, const std::vector<btVector3>& hullPoints, PhysicsTaskScheduler* scheduler);
        ~Object() override;

        bool isDynamic() const { return mRigidBody != nullptr; }

        /// For static objects: collide with simulated objects using this (more detailed) shape instead.
        void setDetailShape(std::shared_ptr<Resource::BulletShapeInstance> shapeInstance);
        btCollisionObject* getDetailCollisionObject() const { return mDetailCollisionObject.get(); }
        btRigidBody* getRigidBody() const { return mRigidBody; }

        /// For dynamic objects: if the simulation moved the body since the last call, return the
        /// new position and rotation of the object's origin (as opposed to its center of mass).
        std::optional<std::pair<osg::Vec3f, osg::Quat>> takeSimulatedTransform();

        /// For dynamic objects: world-space height of the lowest and highest point of the simulated shape.
        std::pair<float, float> getDynamicShapeHeightRange() const;
        /// For dynamic objects: the simulated shape, and where it is for the object's current position.
        btCollisionShape* getDynamicShape() const { return mDynamicShape.get(); }
        btTransform getCenterOfMassTransform() const;

        /// Move the object; takes effect on the next commitPositionChange.
        void moveBy(const osg::Vec3f& offset);

        /// For dynamic objects: where it was last placed by the game (not the simulation).
        osg::Vec3f getPlacedPosition() const;

        const std::shared_ptr<Resource::BulletShapeInstance>& getShapeInstance() const;
        void setScale(float scale);
        void setRotation(osg::Quat quat);
        void updatePosition();
        void commitPositionChange();
        btTransform getTransform() const;
        /// Return solid flag. Not used by the object itself, true by default.
        bool isSolid() const;
        void setSolid(bool solid);
        bool isAnimated() const;
        /// @brief update object shape
        /// @return true if shape changed
        bool animateCollisionShapes();
        bool collidedWith(ScriptedCollisionType type) const;
        void addCollision(ScriptedCollisionType type);
        void resetCollisions();

    private:
        std::shared_ptr<Resource::BulletShapeInstance> mShapeInstance;
        std::map<int, osg::NodePath> mRecordIndexToNodePath;
        bool mSolid;
        btVector3 mScale;
        osg::Vec3f mPosition;
        osg::Quat mRotation;
        bool mScaleUpdatePending = false;
        bool mTransformUpdatePending = false;
        mutable std::mutex mPositionMutex;
        PhysicsTaskScheduler* mTaskScheduler;
        char mCollidedWith;

        // Static objects only
        std::shared_ptr<Resource::BulletShapeInstance> mDetailShapeInstance;
        std::unique_ptr<btCollisionObject> mDetailCollisionObject;

        // Dynamic objects only
        std::unique_ptr<btCollisionShape> mDynamicShape;
        std::unique_ptr<DynamicMotionState> mMotionState;
        btRigidBody* mRigidBody = nullptr; // same object as mCollisionObject
        btVector3 mCenterOffset; // center of mass relative to object origin, in object space
        osg::Vec3f mPlacedPosition;
    };
}

#endif
