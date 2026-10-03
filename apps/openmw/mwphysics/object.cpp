#include "object.hpp"
#include "mtphysics.hpp"
#include <algorithm>
#include <memory>

#include <components/bullethelpers/collisionobject.hpp>
#include <components/debug/debuglog.hpp>
#include <components/misc/constants.hpp>
#include <components/misc/convert.hpp>
#include <components/nifosg/particle.hpp>
#include <components/resource/bulletshape.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>

#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

#include <LinearMath/btMotionState.h>
#include <LinearMath/btTransform.h>

namespace MWPhysics
{
    // Receives body transforms from Bullet on the physics thread; the main thread picks them up.
    class DynamicMotionState final : public btMotionState
    {
    public:
        explicit DynamicMotionState(const btTransform& transform)
            : mTransform(transform)
        {
        }

        void getWorldTransform(btTransform& transform) const override
        {
            std::lock_guard lock(mMutex);
            transform = mTransform;
        }

        void setWorldTransform(const btTransform& transform) override
        {
            std::lock_guard lock(mMutex);
            mTransform = transform;
            mDirty = true;
        }

        std::optional<btTransform> take()
        {
            std::lock_guard lock(mMutex);
            if (!mDirty)
                return std::nullopt;
            mDirty = false;
            return mTransform;
        }

        void reset(const btTransform& transform)
        {
            std::lock_guard lock(mMutex);
            mTransform = transform;
            mDirty = false;
        }

    private:
        mutable std::mutex mMutex;
        btTransform mTransform;
        bool mDirty = false;
    };

    Object::Object(const MWWorld::Ptr& ptr, std::shared_ptr<Resource::BulletShapeInstance> shapeInstance,
        osg::Quat rotation, int collisionType, PhysicsTaskScheduler* scheduler)
        : PtrHolder(ptr, osg::Vec3f())
        , mShapeInstance(std::move(shapeInstance))
        , mSolid(true)
        , mScale(ptr.getCellRef().getScale(), ptr.getCellRef().getScale(), ptr.getCellRef().getScale())
        , mPosition(ptr.getRefData().getPosition().asVec3())
        , mRotation(rotation)
        , mTaskScheduler(scheduler)
        , mCollidedWith(ScriptedCollisionType_None)
    {
        mCollisionObject = BulletHelpers::makeCollisionObject(mShapeInstance->mCollisionShape.get(),
            Misc::Convert::toBullet(mPosition), Misc::Convert::toBullet(rotation));
        mCollisionObject->setUserPointer(this);
        mShapeInstance->setLocalScaling(mScale);
        mCollisionObject->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
        // Static, so never "awake": the dynamics step skips pairs where both objects sleep.
        mCollisionObject->setActivationState(ISLAND_SLEEPING);
        mTaskScheduler->addCollisionObject(mCollisionObject.get(), collisionType,
            CollisionType_Actor | CollisionType_HeightMap | CollisionType_Projectile | CollisionType_Dynamic);
    }

    Object::Object(const MWWorld::Ptr& ptr, std::shared_ptr<Resource::BulletShapeInstance> shapeInstance,
        osg::Quat rotation, float mass, bool metal, const std::vector<btVector3>& hullPoints,
        PhysicsTaskScheduler* scheduler)
        : PtrHolder(ptr, osg::Vec3f())
        , mShapeInstance(std::move(shapeInstance))
        , mSolid(true)
        , mScale(ptr.getCellRef().getScale(), ptr.getCellRef().getScale(), ptr.getCellRef().getScale())
        , mPosition(ptr.getRefData().getPosition().asVec3())
        , mRotation(rotation)
        , mTaskScheduler(scheduler)
        , mCollidedWith(ScriptedCollisionType_None)
    {
        mShapeInstance->setLocalScaling(mScale);
        btVector3 aabbMin;
        btVector3 aabbMax;
        if (hullPoints.empty())
            mShapeInstance->mCollisionShape->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
        else
        {
            aabbMin = aabbMax = hullPoints.front() * mScale;
            for (const btVector3& point : hullPoints)
            {
                aabbMin.setMin(point * mScale);
                aabbMax.setMax(point * mScale);
            }
        }
        // The center of the bounding box stands in for the center of mass.
        mCenterOffset = (aabbMin + aabbMax) * 0.5;
        btVector3 halfExtents = (aabbMax - aabbMin) * 0.5;

        // Very thin items (paper, scrolls) get a box with some thickness: a flat hull would be degenerate.
        constexpr btScalar minHullHalfExtent = 1.f;
        if (!hullPoints.empty() && halfExtents[halfExtents.minAxis()] >= minHullHalfExtent)
        {
            auto hull = std::make_unique<btConvexHullShape>();
            for (const btVector3& point : hullPoints)
                hull->addPoint(point * mScale - mCenterOffset, false);
            // Thin hulls (blades, plates) get a thicker skin, so they cannot slip through the ground.
            hull->setMargin(std::clamp(2.5f - static_cast<float>(halfExtents[halfExtents.minAxis()]), 0.3f, 1.5f));
            hull->recalcLocalAabb();
            mDynamicShape = std::move(hull);
        }
        else
        {
            halfExtents.setMax(btVector3(minHullHalfExtent, minHullHalfExtent, minHullHalfExtent));
            mDynamicShape = std::make_unique<btBoxShape>(halfExtents);
        }

        // Density relative to water, treating the weight as kilograms and the shape as about 60% solid.
        const btScalar unitsPerMeter = Constants::UnitsPerMeter;
        const btScalar volume = 8 * halfExtents.x() * halfExtents.y() * halfExtents.z() * 0.6f
            / (unitsPerMeter * unitsPerMeter * unitsPerMeter); // cubic meters
        // Even light things (an empty bottle) float partly submerged, not perched on top.
        mRelativeDensity = std::clamp(static_cast<float>(mass / (volume * 1000)), 0.4f, 8.f);
        if (metal)
            mRelativeDensity = std::max(mRelativeDensity, 3.f);

        btVector3 inertia(0, 0, 0);
        mDynamicShape->calculateLocalInertia(mass, inertia);

        const btTransform centerOfMass = getTransform() * btTransform(btQuaternion::getIdentity(), mCenterOffset);
        mMotionState = std::make_unique<DynamicMotionState>(centerOfMass);
        mPlacedPosition = mPosition;

        btRigidBody::btRigidBodyConstructionInfo info(mass, mMotionState.get(), mDynamicShape.get(), inertia);
        info.m_friction = 0.6f;
        info.m_rollingFriction = 0.01f;
        info.m_restitution = 0.15f;
        info.m_linearDamping = 0.05f;
        info.m_angularDamping = 0.2f;
        // Bullet's defaults assume meters; these are in game units (~70 per meter).
        info.m_linearSleepingThreshold = 4.f;
        info.m_angularSleepingThreshold = 1.f;

        auto body = std::make_unique<btRigidBody>(info);
        mRigidBody = body.get();
        const btScalar minHalfExtent = std::max(halfExtents[halfExtents.minAxis()], minHullHalfExtent);
        mRigidBody->setCcdMotionThreshold(minHalfExtent * 0.5f);
        mRigidBody->setCcdSweptSphereRadius(minHalfExtent);
        // Everything starts asleep, so it stays exactly where it was (as placed in the game files, or as left
        // when the game was saved) until something touches it. Objects created during play are woken separately.
        mRigidBody->setActivationState(ISLAND_SLEEPING);
        mRigidBody->setUserPointer(this);
        mCollisionObject = std::move(body);

        mTaskScheduler->addRigidBody(mRigidBody, CollisionType_Dynamic,
            CollisionType_DynamicSupport | CollisionType_Actor | CollisionType_Dynamic | CollisionType_Projectile);
    }

    Object::~Object()
    {
        if (mDetailCollisionObject != nullptr)
            mTaskScheduler->removeCollisionObject(mDetailCollisionObject.get());
        mTaskScheduler->removeCollisionObject(mCollisionObject.get());
    }

    void Object::setDetailShape(std::shared_ptr<Resource::BulletShapeInstance> shapeInstance)
    {
        mDetailShapeInstance = std::move(shapeInstance);
        mDetailShapeInstance->setLocalScaling(mScale);
        const btTransform transform = getTransform();
        mDetailCollisionObject = BulletHelpers::makeCollisionObject(
            mDetailShapeInstance->mCollisionShape.get(), transform.getOrigin(), transform.getRotation());
        mDetailCollisionObject->setUserPointer(this);
        mDetailCollisionObject->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
        mDetailCollisionObject->setActivationState(ISLAND_SLEEPING);
        mTaskScheduler->addCollisionObject(
            mDetailCollisionObject.get(), CollisionType_DynamicDetail, CollisionType_Dynamic);
        // Simulated objects now use the detailed shape instead of the regular one.
        mTaskScheduler->setCollisionFilterMask(
            mCollisionObject.get(), CollisionType_Actor | CollisionType_HeightMap | CollisionType_Projectile);
    }

    btTransform Object::getCenterOfMassTransform() const
    {
        return getTransform() * btTransform(btQuaternion::getIdentity(), mCenterOffset);
    }

    std::pair<float, float> Object::getDynamicShapeHeightRange() const
    {
        btVector3 aabbMin;
        btVector3 aabbMax;
        mDynamicShape->getAabb(getCenterOfMassTransform(), aabbMin, aabbMax);
        return { static_cast<float>(aabbMin.z()), static_cast<float>(aabbMax.z()) };
    }

    void Object::requestWake()
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mPendingActivation = PendingActivation::Wake;
    }

    void Object::requestSleep()
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mPendingActivation = PendingActivation::Sleep;
    }

    void Object::requestVelocity(const osg::Vec3f& velocity)
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mPendingVelocity = velocity;
    }

    osg::Vec3f Object::getPlacedPosition() const
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        return mPlacedPosition;
    }

    void Object::moveBy(const osg::Vec3f& offset)
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mPosition += offset;
        mTransformUpdatePending = true;
    }

    std::optional<std::pair<osg::Vec3f, osg::Quat>> Object::takeSimulatedTransform()
    {
        if (mMotionState == nullptr)
            return std::nullopt;
        const std::optional<btTransform> centerOfMass = mMotionState->take();
        if (!centerOfMass)
            return std::nullopt;
        const btTransform origin = *centerOfMass * btTransform(btQuaternion::getIdentity(), -mCenterOffset);

        std::unique_lock<std::mutex> lock(mPositionMutex);
        mPosition = Misc::Convert::toOsg(origin.getOrigin());
        mRotation = Misc::Convert::toOsg(origin.getRotation());
        return std::make_pair(mPosition, mRotation);
    }

    const std::shared_ptr<Resource::BulletShapeInstance>& Object::getShapeInstance() const
    {
        return mShapeInstance;
    }

    void Object::setScale(float scale)
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mScale = { scale, scale, scale };
        mScaleUpdatePending = true;
    }

    void Object::setRotation(osg::Quat quat)
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mRotation = quat;
        mTransformUpdatePending = true;
    }

    void Object::updatePosition()
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mPosition = mPtr.getRefData().getPosition().asVec3();
        mTransformUpdatePending = true;
    }

    void Object::commitPositionChange()
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        if (mScaleUpdatePending)
        {
            // Dynamic objects keep the shape they were created with.
            if (mRigidBody == nullptr)
            {
                mShapeInstance->setLocalScaling(mScale);
                if (mDetailShapeInstance != nullptr)
                    mDetailShapeInstance->setLocalScaling(mScale);
            }
            mScaleUpdatePending = false;
        }
        if (mTransformUpdatePending)
        {
            btTransform trans;
            trans.setOrigin(Misc::Convert::toBullet(mPosition));
            trans.setRotation(Misc::Convert::toBullet(mRotation));
            if (mRigidBody != nullptr)
            {
                // Moved by something other than the simulation (e.g. a script): teleport the body there at rest.
                const btTransform centerOfMass = trans * btTransform(btQuaternion::getIdentity(), mCenterOffset);
                mRigidBody->setWorldTransform(centerOfMass);
                mRigidBody->setInterpolationWorldTransform(centerOfMass);
                mRigidBody->setLinearVelocity(btVector3(0, 0, 0));
                mRigidBody->setAngularVelocity(btVector3(0, 0, 0));
                mRigidBody->setInterpolationLinearVelocity(btVector3(0, 0, 0));
                mRigidBody->setInterpolationAngularVelocity(btVector3(0, 0, 0));
                mRigidBody->clearForces();
                // Marks the transform as changed, so the game object follows if the move came from moveBy.
                mMotionState->setWorldTransform(centerOfMass);
                mPlacedPosition = mPosition;
                mRigidBody->activate(true);
            }
            else
            {
                mCollisionObject->setWorldTransform(trans);
                if (mDetailCollisionObject != nullptr)
                    mDetailCollisionObject->setWorldTransform(trans);
            }
            mTransformUpdatePending = false;
        }
        if (mRigidBody != nullptr)
        {
            if (mPendingVelocity)
            {
                mRigidBody->setLinearVelocity(Misc::Convert::toBullet(*mPendingVelocity));
                mPendingVelocity.reset();
            }
            if (mPendingActivation == PendingActivation::Wake)
                mRigidBody->activate(true);
            else if (mPendingActivation == PendingActivation::Sleep)
            {
                mRigidBody->setLinearVelocity(btVector3(0, 0, 0));
                mRigidBody->setAngularVelocity(btVector3(0, 0, 0));
                mRigidBody->forceActivationState(ISLAND_SLEEPING);
            }
            mPendingActivation = PendingActivation::None;
        }
    }

    btTransform Object::getTransform() const
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        btTransform trans;
        trans.setOrigin(Misc::Convert::toBullet(mPosition));
        trans.setRotation(Misc::Convert::toBullet(mRotation));
        return trans;
    }

    bool Object::isSolid() const
    {
        return mSolid;
    }

    void Object::setSolid(bool solid)
    {
        mSolid = solid;
    }

    bool Object::isAnimated() const
    {
        return mShapeInstance->isAnimated();
    }

    bool Object::animateCollisionShapes()
    {
        if (mShapeInstance->mAnimatedShapes.empty())
            return false;

        if (!mPtr.getRefData().getBaseNode())
            return false;

        assert(mShapeInstance->mCollisionShape->isCompound());

        btCompoundShape* compound = static_cast<btCompoundShape*>(mShapeInstance->mCollisionShape.get());
        bool result = false;
        for (const auto& [recordIndex, shapeIndex] : mShapeInstance->mAnimatedShapes)
        {
            auto nodePathFound = mRecordIndexToNodePath.find(recordIndex);
            if (nodePathFound == mRecordIndexToNodePath.end())
            {
                NifOsg::FindGroupByRecordIndex visitor(recordIndex);
                mPtr.getRefData().getBaseNode()->accept(visitor);
                if (!visitor.mFound)
                {
                    Log(Debug::Warning) << "Warning: animateCollisionShapes can't find node " << recordIndex << " for "
                                        << mPtr.getCellRef().getRefId();

                    // Remove nonexistent nodes from animated shapes map and early out
                    mShapeInstance->mAnimatedShapes.erase(recordIndex);
                    return false;
                }
                osg::NodePath nodePath = visitor.mFoundPath;
                nodePath.erase(nodePath.begin());
                nodePathFound = mRecordIndexToNodePath.emplace(recordIndex, nodePath).first;
            }

            osg::NodePath& nodePath = nodePathFound->second;
            osg::Matrixf matrix = osg::computeLocalToWorld(nodePath);
            btVector3 scale = Misc::Convert::toBullet(matrix.getScale());
            matrix.orthoNormalize(matrix);

            btTransform transform;
            transform.setOrigin(Misc::Convert::toBullet(matrix.getTrans()) * compound->getLocalScaling());
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    transform.getBasis()[i][j] = matrix(j, i); // NB column/row major difference

            btCollisionShape* childShape = compound->getChildShape(shapeIndex);
            btVector3 newScale = compound->getLocalScaling() * scale;

            if (childShape->getLocalScaling() != newScale)
            {
                childShape->setLocalScaling(newScale);
                result = true;
            }

            if (!(transform == compound->getChildTransform(shapeIndex)))
            {
                compound->updateChildTransform(shapeIndex, transform);
                result = true;
            }
        }
        return result;
    }

    bool Object::collidedWith(ScriptedCollisionType type) const
    {
        return mCollidedWith & type;
    }

    void Object::addCollision(ScriptedCollisionType type)
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mCollidedWith |= type;
    }

    void Object::resetCollisions()
    {
        mCollidedWith = ScriptedCollisionType_None;
    }
}
