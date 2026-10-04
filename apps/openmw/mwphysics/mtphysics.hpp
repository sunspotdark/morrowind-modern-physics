#ifndef OPENMW_MWPHYSICS_MTPHYSICS_H
#define OPENMW_MWPHYSICS_MTPHYSICS_H

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <shared_mutex>
#include <thread>
#include <unordered_set>

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

class btDiscreteDynamicsWorld;
class btRigidBody;
class btTypedConstraint;
class btActionInterface;
class btPoint2PointConstraint;

#include <osg/Timer>

#include "components/misc/budgetmeasurement.hpp"
#include "physicssystem.hpp"
#include "ptrholder.hpp"

namespace Misc
{
    class Barrier;
}

namespace MWRender
{
    class DebugDrawer;
}

namespace MWPhysics
{
    enum class LockingPolicy
    {
        NoLocks,
        ExclusiveLocksOnly,
        AllowSharedLocks,
    };

    class PhysicsTaskScheduler
    {
    public:
        PhysicsTaskScheduler(
            float physicsDt, btDiscreteDynamicsWorld* dynamicsWorld, MWRender::DebugDrawer* debugDrawer);
        ~PhysicsTaskScheduler();

        /// @brief move actors taking into account desired movements and collisions
        /// @param numSteps how much simulation step to run
        /// @param timeAccum accumulated time from previous run to interpolate movements
        /// @param actorsData per actor data needed to compute new positions
        /// @return new position of each actor
        void applyQueuedMovements(float& timeAccum, std::vector<Simulation>& simulations, osg::Timer_t frameStart,
            unsigned int frameNumber, osg::Stats& stats);

        void resetSimulation(const ActorMap& actors);

        // Thread safe wrappers
        void rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld,
            btCollisionWorld::RayResultCallback& resultCallback) const;
        void convexSweepTest(const btConvexShape* castShape, const btTransform& from, const btTransform& to,
            btCollisionWorld::ConvexResultCallback& resultCallback) const;
        void contactTest(btCollisionObject* colObj, btCollisionWorld::ContactResultCallback& resultCallback);
        std::optional<btVector3> getHitPoint(const btTransform& from, btCollisionObject* target);
        void aabbTest(const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback);
        void getAabb(const btCollisionObject* obj, btVector3& min, btVector3& max);
        void setCollisionFilterMask(btCollisionObject* collisionObject, int collisionFilterMask);
        void addCollisionObject(btCollisionObject* collisionObject, int collisionFilterGroup, int collisionFilterMask);
        void addRigidBody(btRigidBody* body, int collisionFilterGroup, int collisionFilterMask);
        /// A joint between two rigid bodies already added (its bodies don't collide with each other).
        void addConstraint(btTypedConstraint* constraint);
        void removeConstraint(btTypedConstraint* constraint);
        /// Something run every simulation step (inside it, with the world locked).
        void addAction(btActionInterface* action);
        void removeAction(btActionInterface* action);
        /// Stop a body and put it to sleep.
        void setAtRest(btRigidBody* body);

        // Carrying a dynamic object: each physics step it is steered towards the hold target.
        /// steerRotation: also turn it to the target orientation. holdMass: if set, its mass while held (a heavier
        /// grip drags what it is jointed to, e.g. the rest of a ragdoll).
        /// grabPoint: if set, it isn't carried but hangs from this world point (pinned there), which then follows
        /// the hold target.
        void holdObject(const std::shared_ptr<PtrHolder>& object, bool steerRotation, btScalar holdMass,
            const std::optional<btVector3>& grabPoint = std::nullopt);
        void setHoldTarget(const btVector3& position, const btQuaternion& rotation);
        /// Let go of the held object, optionally setting its velocity (for throwing).
        void releaseHeldObject(const std::optional<btVector3>& velocity);
        std::shared_ptr<PtrHolder> getHeldObject() const;

        /// Change the velocity of dynamic objects as if struck at a world point (velocityChange is independent
        /// of the object's mass). Ignores the held object.
        struct Strike
        {
            std::shared_ptr<Object> mObject;
            btVector3 mVelocityChange;
            btVector3 mPoint;
            btVector3 mSource; // where the blow came from
        };
        void strikeObjects(const std::vector<Strike>& strikes);

        /// Water level for buoyancy, or none.
        void setWaterHeight(std::optional<float> height);

        /// Watch a thrown object for hitting an actor (other than the thrower) while it flies.
        void addFlyingObject(const std::shared_ptr<Object>& object, const btCollisionObject* thrower);
        struct FlyingObjectHit
        {
            std::weak_ptr<Object> mObject;
            const btCollisionObject* mActor;
            btScalar mSpeed;
        };
        /// Hits since the last call.
        std::vector<FlyingObjectHit> takeFlyingObjectHits();
        void removeCollisionObject(btCollisionObject* collisionObject);
        void updateSingleAabb(const std::shared_ptr<PtrHolder>& ptr, bool immediate = false);
        bool getLineOfSight(const std::shared_ptr<Actor>& actor1, const std::shared_ptr<Actor>& actor2);
        void debugDraw();
        void* getUserPointer(const btCollisionObject* object) const;
        void releaseSharedStates(); // destroy all objects whose destructor can't be safely called from
                                    // ~PhysicsTaskScheduler()

    private:
        class WorkersSync;

        void doSimulation();
        void worker();
        void updateActorsPositions();
        bool hasLineOfSight(const Actor* actor1, const Actor* actor2);
        void refreshLOSCache();
        void updateAabbs();
        void updatePtrAabb(const std::shared_ptr<PtrHolder>& ptr);
        void updateStats(osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats);
        std::tuple<unsigned, float> calculateStepConfig(float timeAccum) const;
        void afterPreStep();
        void afterPostStep();
        void afterPostSim();
        void pushDynamicObjects();
        void stepDynamics();
        void releaseHeldObjectUnsafe(btRigidBody& body, const std::optional<btVector3>& velocity);
        void applyWaterForces(float waterHeight);
        void freeWedgedObjectUnsafe(const std::shared_ptr<Object>& object);
        void setStuckUnsafe(Object& object, bool stuck);
        void updateWedgedObjects(const std::vector<std::shared_ptr<Object>>& objects);
        struct FlyingObject
        {
            std::weak_ptr<Object> mObject;
            const btCollisionObject* mThrower;
        };
        void updateFlyingObjects(const std::vector<std::pair<std::shared_ptr<Object>, FlyingObject>>& objects);
        void syncWithMainThread();
        void waitForWorkers();
        void prepareWork(float& timeAccum, std::vector<Simulation>& simulations, osg::Timer_t frameStart,
            unsigned int frameNumber, osg::Stats& stats);

        std::unique_ptr<WorldFrameData> mWorldFrameData;
        std::vector<Simulation>* mSimulations = nullptr;
        std::unordered_set<const btCollisionObject*> mCollisionObjects;
        float mDefaultPhysicsDt;
        float mPhysicsDt;
        float mTimeAccum;
        btCollisionWorld* mCollisionWorld;
        btDiscreteDynamicsWorld* mDynamicsWorld; // same object as mCollisionWorld
        int mNumRigidBodies = 0;
        std::atomic<bool> mHasWater{ false };
        std::atomic<float> mWaterHeight{ 0.f };
        std::weak_ptr<PtrHolder> mHeldObject;
        bool mHoldSteerRotation = true;
        btScalar mHeldRestoreMass = 0; // the mass to give back on release, if changed while held
        std::unique_ptr<btPoint2PointConstraint> mHoldPin; // see holdObject
        void removeHoldPinUnsafe();
        btVector3 mHoldTarget;
        btQuaternion mHoldTargetRotation;
        // Dynamic objects passing through statics they were wedged in (see freeWedgedObjectUnsafe).
        std::vector<std::weak_ptr<Object>> mWedgedObjects;
        std::mutex mWedgedObjectsMutex;
        // Thrown objects still flying, and the actors they hit (see addFlyingObject).
        std::vector<FlyingObject> mFlyingObjects;
        std::vector<FlyingObjectHit> mFlyingObjectHits;
        std::mutex mFlyingObjectsMutex;
        mutable std::mutex mHeldObjectMutex;
        MWRender::DebugDrawer* mDebugDrawer;
        std::vector<LOSRequest> mLOSCache;
        std::set<std::weak_ptr<PtrHolder>, std::owner_less<std::weak_ptr<PtrHolder>>> mUpdateAabb;

        // TODO: use std::experimental::flex_barrier or std::barrier once it becomes a thing
        std::unique_ptr<Misc::Barrier> mPreStepBarrier;
        std::unique_ptr<Misc::Barrier> mPostStepBarrier;
        std::unique_ptr<Misc::Barrier> mPostSimBarrier;

        LockingPolicy mLockingPolicy;
        unsigned mNumThreads;
        int mNumJobs;
        unsigned mRemainingSteps;
        int mLOSCacheExpiry;
        bool mAdvanceSimulation;
        std::atomic<int> mNextJob;
        std::atomic<int> mNextLOS;
        std::vector<std::thread> mThreads;

        mutable std::shared_mutex mSimulationMutex;
        mutable std::shared_mutex mCollisionWorldMutex;
        mutable std::shared_mutex mLOSCacheMutex;
        mutable std::mutex mUpdateAabbMutex;

        unsigned int mFrameNumber;
        const osg::Timer* mTimer;

        unsigned mPrevStepCount;
        Misc::BudgetMeasurement mBudget;
        Misc::BudgetMeasurement mAsyncBudget;
        unsigned int mBudgetCursor;
        osg::Timer_t mAsyncStartTime;
        osg::Timer_t mTimeBegin;
        osg::Timer_t mTimeEnd;
        osg::Timer_t mFrameStart;

        std::unique_ptr<WorkersSync> mWorkersSync;
    };

}
#endif
