#include "mtphysics.hpp"

#include <algorithm>
#include <cassert>
#include <functional>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <stdexcept>
#include <variant>

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btThreads.h>

#include <osg/Stats>

#include "components/debug/debuglog.hpp"
#include "components/misc/convert.hpp"
#include <components/misc/barrier.hpp>
#include <components/settings/values.hpp>

#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/creaturestats.hpp"

#include "../mwrender/bulletdebugdraw.hpp"

#include "../mwworld/class.hpp"

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"

#include "actor.hpp"
#include "contacttestwrapper.h"
#include "movementsolver.hpp"
#include "object.hpp"
#include "physicssystem.hpp"
#include "projectile.hpp"

namespace MWPhysics
{
    namespace
    {
        template <class Mutex>
        std::optional<std::unique_lock<Mutex>> makeExclusiveLock(Mutex& mutex, LockingPolicy lockingPolicy)
        {
            if (lockingPolicy == LockingPolicy::NoLocks)
                return {};
            return std::unique_lock(mutex);
        }

        /// @brief A scoped lock that is either exclusive or inexistent depending on configuration
        template <class Mutex>
        class MaybeExclusiveLock
        {
        public:
            /// @param mutex a mutex
            /// @param threadCount decide wether the excluse lock will be taken
            explicit MaybeExclusiveLock(Mutex& mutex, LockingPolicy lockingPolicy)
                : mImpl(makeExclusiveLock(mutex, lockingPolicy))
            {
            }

        private:
            std::optional<std::unique_lock<Mutex>> mImpl;
        };

        template <class Mutex>
        std::optional<std::shared_lock<Mutex>> makeSharedLock(Mutex& mutex, LockingPolicy lockingPolicy)
        {
            if (lockingPolicy == LockingPolicy::NoLocks)
                return {};
            return std::shared_lock(mutex);
        }

        /// @brief A scoped lock that is either shared or inexistent depending on configuration
        template <class Mutex>
        class MaybeSharedLock
        {
        public:
            /// @param mutex a shared mutex
            /// @param threadCount decide wether the shared lock will be taken
            explicit MaybeSharedLock(Mutex& mutex, LockingPolicy lockingPolicy)
                : mImpl(makeSharedLock(mutex, lockingPolicy))
            {
            }

        private:
            std::optional<std::shared_lock<Mutex>> mImpl;
        };

        template <class Mutex>
        std::variant<std::monostate, std::unique_lock<Mutex>, std::shared_lock<Mutex>> makeLock(
            Mutex& mutex, LockingPolicy lockingPolicy)
        {
            switch (lockingPolicy)
            {
                case LockingPolicy::NoLocks:
                    return std::monostate{};
                case LockingPolicy::ExclusiveLocksOnly:
                    return std::unique_lock(mutex);
                case LockingPolicy::AllowSharedLocks:
                    return std::shared_lock(mutex);
            };

            throw std::runtime_error("Unsupported LockingPolicy: "
                + std::to_string(static_cast<std::underlying_type_t<LockingPolicy>>(lockingPolicy)));
        }

        /// @brief A scoped lock that is either shared, exclusive or inexistent depending on configuration
        template <class Mutex>
        class MaybeLock
        {
        public:
            /// @param mutex a shared mutex
            /// @param threadCount decide wether the lock will be shared, exclusive or inexistent
            explicit MaybeLock(Mutex& mutex, LockingPolicy lockingPolicy)
                : mImpl(makeLock(mutex, lockingPolicy))
            {
            }

        private:
            std::variant<std::monostate, std::unique_lock<Mutex>, std::shared_lock<Mutex>> mImpl;
        };
    }
}

namespace
{
    bool isUnderWater(const MWPhysics::ActorFrameData& actorData)
    {
        return actorData.mPosition.z() < actorData.mSwimLevel;
    }

    // Finds the dynamic bodies an actor overlaps, keeping the deepest contact for each.
    class DynamicContactCallback final : public btCollisionWorld::ContactResultCallback
    {
    public:
        struct Contact
        {
            btRigidBody* mBody;
            btVector3 mPoint; // on the body, world space
            btVector3 mPushDirection; // from the actor into the body
            btScalar mDistance;
        };

        explicit DynamicContactCallback(const btCollisionObject* actor)
            : mActor(actor)
        {
            m_collisionFilterGroup = MWPhysics::CollisionType_Actor;
            m_collisionFilterMask = MWPhysics::CollisionType_Dynamic;
        }

        btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* col0Wrap, int /*partId0*/,
            int /*index0*/, const btCollisionObjectWrapper* col1Wrap, int /*partId1*/, int /*index1*/) override
        {
            // m_normalWorldOnB points from B towards A.
            const bool actorIsA = col0Wrap->getCollisionObject() == mActor;
            const btCollisionObject* other
                = actorIsA ? col1Wrap->getCollisionObject() : col0Wrap->getCollisionObject();
            btRigidBody* body = btRigidBody::upcast(const_cast<btCollisionObject*>(other));
            if (body == nullptr)
                return 0;
            const btVector3 point = actorIsA ? cp.getPositionWorldOnB() : cp.getPositionWorldOnA();
            const btVector3 pushDirection = actorIsA ? -cp.m_normalWorldOnB : cp.m_normalWorldOnB;

            auto existing = std::find_if(
                mContacts.begin(), mContacts.end(), [&](const Contact& c) { return c.mBody == body; });
            if (existing == mContacts.end())
                mContacts.push_back({ body, point, pushDirection, cp.getDistance() });
            else if (cp.getDistance() < existing->mDistance)
                *existing = { body, point, pushDirection, cp.getDistance() };
            return 0;
        }

        std::vector<Contact> mContacts;

    private:
        const btCollisionObject* mActor;
    };

    // Finds the static objects a body penetrates deeper than a given depth.
    class PenetrationCallback final : public btCollisionWorld::ContactResultCallback
    {
    public:
        PenetrationCallback(const btCollisionObject* body, btScalar minDepth)
            : mBody(body)
            , mMinDepth(minDepth)
        {
            m_collisionFilterGroup = MWPhysics::CollisionType_Dynamic;
            m_collisionFilterMask = MWPhysics::CollisionType_World | MWPhysics::CollisionType_Door
                | MWPhysics::CollisionType_DynamicDetail;
        }

        btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* col0Wrap, int /*partId0*/,
            int /*index0*/, const btCollisionObjectWrapper* col1Wrap, int /*partId1*/, int /*index1*/) override
        {
            const btCollisionObject* other = col0Wrap->getCollisionObject() == mBody
                ? col1Wrap->getCollisionObject()
                : col0Wrap->getCollisionObject();
            if (cp.getDistance() < -mMinDepth && std::find(mOthers.begin(), mOthers.end(), other) == mOthers.end())
                mOthers.push_back(other);
            return 0;
        }

        std::vector<const btCollisionObject*> mOthers;

    private:
        const btCollisionObject* mBody;
        btScalar mMinDepth;
    };

    // Finds an actor (other than the thrower) a thrown body is touching, and the contact normal towards the body.
    class ActorHitCallback final : public btCollisionWorld::ContactResultCallback
    {
    public:
        ActorHitCallback(const btCollisionObject* body, const btCollisionObject* thrower)
            : mBody(body)
            , mThrower(thrower)
        {
            // Actors don't collide with simulated objects, so pose as an actor to get past their filter.
            m_collisionFilterGroup = MWPhysics::CollisionType_Actor;
            m_collisionFilterMask = MWPhysics::CollisionType_Actor;
        }

        btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* col0Wrap, int /*partId0*/,
            int /*index0*/, const btCollisionObjectWrapper* col1Wrap, int /*partId1*/, int /*index1*/) override
        {
            // m_normalWorldOnB points from B towards A.
            const bool bodyIsA = col0Wrap->getCollisionObject() == mBody;
            const btCollisionObject* other = bodyIsA ? col1Wrap->getCollisionObject() : col0Wrap->getCollisionObject();
            if (mActor == nullptr && other != mThrower && cp.getDistance() < 0)
            {
                mActor = other;
                mNormal = bodyIsA ? cp.m_normalWorldOnB : -cp.m_normalWorldOnB;
            }
            return 0;
        }

        const btCollisionObject* mActor = nullptr;
        btVector3 mNormal{ 0, 0, 1 };

    private:
        const btCollisionObject* mBody;
        const btCollisionObject* mThrower;
    };

    // Finds a piece of furniture (a detailed furniture collision object) a body is resting on or in.
    class FurnitureContactCallback final : public btCollisionWorld::ContactResultCallback
    {
    public:
        explicit FurnitureContactCallback(const btCollisionObject* body)
            : mBody(body)
        {
            m_collisionFilterGroup = MWPhysics::CollisionType_Dynamic;
            m_collisionFilterMask = MWPhysics::CollisionType_DynamicDetail;
            m_closestDistanceThreshold = 2.f; // resting counts, not just overlapping
        }

        btScalar addSingleResult(btManifoldPoint& /*cp*/, const btCollisionObjectWrapper* col0Wrap, int /*partId0*/,
            int /*index0*/, const btCollisionObjectWrapper* col1Wrap, int /*partId1*/, int /*index1*/) override
        {
            if (mFurniture == nullptr)
                mFurniture = col0Wrap->getCollisionObject() == mBody ? col1Wrap->getCollisionObject()
                                                                      : col0Wrap->getCollisionObject();
            return 0;
        }

        const btCollisionObject* mFurniture = nullptr;

    private:
        const btCollisionObject* mBody;
    };

    osg::Vec3f interpolateMovements(const MWPhysics::PtrHolder& ptr, float timeAccum, float physicsDt)
    {
        const float interpolationFactor = std::clamp(timeAccum / physicsDt, 0.0f, 1.0f);
        return ptr.getPosition() * interpolationFactor + ptr.getPreviousPosition() * (1.f - interpolationFactor);
    }

    using LockedActorSimulation
        = std::pair<std::shared_ptr<MWPhysics::Actor>, std::reference_wrapper<MWPhysics::ActorFrameData>>;
    using LockedProjectileSimulation
        = std::pair<std::shared_ptr<MWPhysics::Projectile>, std::reference_wrapper<MWPhysics::ProjectileFrameData>>;

    namespace Visitors
    {
        template <class Impl, template <class> class Lock>
        struct WithLockedPtr
        {
            const Impl& mImpl;
            std::shared_mutex& mCollisionWorldMutex;
            const MWPhysics::LockingPolicy mLockingPolicy;

            template <class Ptr, class FrameData>
            void operator()(MWPhysics::SimulationImpl<Ptr, FrameData>& sim) const
            {
                auto locked = sim.lock();
                if (!locked.has_value())
                    return;
                auto&& [ptr, frameData] = *std::move(locked);
                // Locked shared_ptr has to be destructed after releasing mCollisionWorldMutex to avoid
                // possible deadlock. Ptr destructor also acquires mCollisionWorldMutex.
                const std::pair arg(std::move(ptr), frameData);
                const Lock<std::shared_mutex> lock(mCollisionWorldMutex, mLockingPolicy);
                mImpl(arg);
            }
        };

        struct InitPosition
        {
            const btCollisionWorld* mCollisionWorld;
            void operator()(MWPhysics::ActorSimulation& sim) const
            {
                auto locked = sim.lock();
                if (!locked.has_value())
                    return;
                auto& [actor, frameDataRef] = *locked;
                auto& frameData = frameDataRef.get();
                frameData.mPosition = actor->applyOffsetChange();
                if (frameData.mWaterCollision && frameData.mPosition.z() < frameData.mWaterlevel
                    && actor->canMoveToWaterSurface(frameData.mWaterlevel, mCollisionWorld))
                {
                    const auto offset = osg::Vec3f(0, 0, frameData.mWaterlevel - frameData.mPosition.z());
                    MWBase::Environment::get().getWorld()->moveObjectBy(actor->getPtr(), offset, false);
                    frameData.mPosition = actor->applyOffsetChange();
                }
                actor->updateCollisionObjectPosition();
                frameData.mOldHeight = frameData.mPosition.z();
                const auto rotation = actor->getPtr().getRefData().getPosition().asRotationVec3();
                frameData.mRotation = osg::Vec2f(rotation.x(), rotation.z());
                frameData.mInertia = actor->getInertialForce();
                frameData.mStuckFrames = actor->getStuckFrames();
                frameData.mLastStuckPosition = actor->getLastStuckPosition();
            }
            void operator()(MWPhysics::ProjectileSimulation& /*sim*/) const {}
        };

        struct PreStep
        {
            btCollisionWorld* mCollisionWorld;
            void operator()(const LockedActorSimulation& sim) const
            {
                MWPhysics::MovementSolver::unstuck(sim.second, mCollisionWorld);
            }
            void operator()(const LockedProjectileSimulation& /*sim*/) const {}
        };

        struct UpdatePosition
        {
            btCollisionWorld* mCollisionWorld;
            void operator()(const LockedActorSimulation& sim) const
            {
                auto& [actor, frameDataRef] = sim;
                auto& frameData = frameDataRef.get();
                if (actor->setPosition(frameData.mPosition))
                {
                    frameData.mPosition = actor->getPosition(); // account for potential position change made by script
                    actor->updateCollisionObjectPosition();
                    mCollisionWorld->updateSingleAabb(actor->getCollisionObject());
                }
            }
            void operator()(const LockedProjectileSimulation& sim) const
            {
                auto& [proj, frameDataRef] = sim;
                auto& frameData = frameDataRef.get();
                proj->setPosition(frameData.mPosition);
                proj->updateCollisionObjectPosition();
                mCollisionWorld->updateSingleAabb(proj->getCollisionObject());
            }
        };

        struct Move
        {
            const float mPhysicsDt;
            const btCollisionWorld* mCollisionWorld;
            const MWPhysics::WorldFrameData& mWorldFrameData;
            void operator()(const LockedActorSimulation& sim) const
            {
                MWPhysics::MovementSolver::move(sim.second, mPhysicsDt, mCollisionWorld, mWorldFrameData);
            }
            void operator()(const LockedProjectileSimulation& sim) const
            {
                if (sim.first->isActive())
                    MWPhysics::MovementSolver::move(sim.second, mPhysicsDt, mCollisionWorld);
            }
        };

        struct Sync
        {
            const bool mAdvanceSimulation;
            const float mTimeAccum;
            const float mPhysicsDt;
            const MWPhysics::PhysicsTaskScheduler* scheduler;
            void operator()(MWPhysics::ActorSimulation& sim) const
            {
                auto locked = sim.lock();
                if (!locked.has_value())
                    return;
                auto& [actor, frameDataRef] = *locked;
                auto& frameData = frameDataRef.get();
                auto ptr = actor->getPtr();

                MWMechanics::CreatureStats& stats = ptr.getClass().getCreatureStats(ptr);
                const float heightDiff = frameData.mPosition.z() - frameData.mOldHeight;
                const bool isStillOnGround = (mAdvanceSimulation && frameData.mWasOnGround && frameData.mIsOnGround);

                if (isStillOnGround || frameData.mFlying || isUnderWater(frameData) || frameData.mSlowFall < 1)
                    stats.land(ptr == MWMechanics::getPlayer() && (frameData.mFlying || isUnderWater(frameData)));
                else if (heightDiff < 0)
                    stats.addToFallHeight(-heightDiff);

                actor->setSimulationPosition(::interpolateMovements(*actor, mTimeAccum, mPhysicsDt));
                actor->setLastStuckPosition(frameData.mLastStuckPosition);
                actor->setStuckFrames(frameData.mStuckFrames);
                if (mAdvanceSimulation)
                {
                    MWWorld::Ptr standingOn;
                    if (frameData.mStandingOn != nullptr)
                    {
                        auto* const ptrHolder
                            = static_cast<MWPhysics::PtrHolder*>(scheduler->getUserPointer(frameData.mStandingOn));
                        if (ptrHolder != nullptr)
                            standingOn = ptrHolder->getPtr();
                    }
                    actor->setStandingOnPtr(standingOn);
                    // the "on ground" state of an actor might have been updated by a traceDown, don't overwrite the
                    // change
                    if (actor->getOnGround() == frameData.mWasOnGround)
                        actor->setOnGround(frameData.mIsOnGround);
                    actor->setOnSlope(frameData.mIsOnSlope);
                    actor->setWalkingOnWater(frameData.mWalkingOnWater);
                    actor->setInertialForce(frameData.mInertia);
                }
            }
            void operator()(MWPhysics::ProjectileSimulation& sim) const
            {
                auto locked = sim.lock();
                if (!locked.has_value())
                    return;
                auto& [proj, frameData] = *locked;
                proj->setSimulationPosition(::interpolateMovements(*proj, mTimeAccum, mPhysicsDt));
            }
        };
    }
}

namespace MWPhysics
{
    namespace
    {
        unsigned getMaxBulletSupportedThreads()
        {
            auto broad = std::make_unique<btDbvtBroadphase>();
            assert(BT_MAX_THREAD_COUNT > 0);
            return std::min<unsigned>(broad->m_rayTestStacks.size(), BT_MAX_THREAD_COUNT - 1);
        }

        LockingPolicy detectLockingPolicy()
        {
            if (Settings::physics().mAsyncNumThreads < 1)
                return LockingPolicy::NoLocks;
            if (getMaxBulletSupportedThreads() > 1)
                return LockingPolicy::AllowSharedLocks;
            Log(Debug::Warning) << "Bullet was not compiled with multithreading support, 1 async thread will be used";
            return LockingPolicy::ExclusiveLocksOnly;
        }

        unsigned getNumThreads(LockingPolicy lockingPolicy)
        {
            switch (lockingPolicy)
            {
                case LockingPolicy::NoLocks:
                    return 0;
                case LockingPolicy::ExclusiveLocksOnly:
                    return 1;
                case LockingPolicy::AllowSharedLocks:
                    return static_cast<unsigned>(std::clamp<int>(
                        Settings::physics().mAsyncNumThreads, 0, static_cast<int>(getMaxBulletSupportedThreads())));
            }

            throw std::runtime_error("Unsupported LockingPolicy: "
                + std::to_string(static_cast<std::underlying_type_t<LockingPolicy>>(lockingPolicy)));
        }
    }

    class PhysicsTaskScheduler::WorkersSync
    {
    public:
        void waitForWorkers()
        {
            std::unique_lock lock(mWorkersDoneMutex);
            if (mFrameCounter != mWorkersFrameCounter)
                mWorkersDone.wait(lock);
        }

        void wakeUpWorkers()
        {
            const std::lock_guard lock(mHasJobMutex);
            ++mFrameCounter;
            mHasJob.notify_all();
        }

        void stopWorkers()
        {
            const std::lock_guard lock(mHasJobMutex);
            mShouldStop = true;
            mHasJob.notify_all();
        }

        void workIsDone()
        {
            const std::lock_guard lock(mWorkersDoneMutex);
            ++mWorkersFrameCounter;
            mWorkersDone.notify_all();
        }

        template <class F>
        void runWorker(F&& f) noexcept
        {
            std::size_t lastFrame = 0;
            std::unique_lock lock(mHasJobMutex);
            while (!mShouldStop)
            {
                mHasJob.wait(lock, [&] { return mShouldStop || mFrameCounter != lastFrame; });
                lastFrame = mFrameCounter;
                lock.unlock();
                f();
                lock.lock();
            }
        }

    private:
        std::size_t mWorkersFrameCounter = 0;
        std::condition_variable mWorkersDone;
        std::mutex mWorkersDoneMutex;
        std::condition_variable mHasJob;
        bool mShouldStop = false;
        std::size_t mFrameCounter = 0;
        std::mutex mHasJobMutex;
    };

    PhysicsTaskScheduler::PhysicsTaskScheduler(
        float physicsDt, btDiscreteDynamicsWorld* dynamicsWorld, MWRender::DebugDrawer* debugDrawer)
        : mDefaultPhysicsDt(physicsDt)
        , mPhysicsDt(physicsDt)
        , mTimeAccum(0.f)
        , mCollisionWorld(dynamicsWorld)
        , mDynamicsWorld(dynamicsWorld)
        , mDebugDrawer(debugDrawer)
        , mLockingPolicy(detectLockingPolicy())
        , mNumThreads(getNumThreads(mLockingPolicy))
        , mNumJobs(0)
        , mRemainingSteps(0)
        , mLOSCacheExpiry(Settings::physics().mLineofsightKeepInactiveCache)
        , mAdvanceSimulation(false)
        , mNextJob(0)
        , mNextLOS(0)
        , mFrameNumber(0)
        , mTimer(osg::Timer::instance())
        , mPrevStepCount(1)
        , mBudget(physicsDt)
        , mAsyncBudget(0.0f)
        , mBudgetCursor(0)
        , mAsyncStartTime(0)
        , mTimeBegin(0)
        , mTimeEnd(0)
        , mFrameStart(0)
        , mWorkersSync(mNumThreads >= 1 ? std::make_unique<WorkersSync>() : nullptr)
    {
        if (mNumThreads >= 1)
        {
            Log(Debug::Info) << "Using " << mNumThreads << " async physics threads";
            for (unsigned i = 0; i < mNumThreads; ++i)
                mThreads.emplace_back([&] { worker(); });
        }
        else
        {
            mLOSCacheExpiry = 0;
        }

        mPreStepBarrier = std::make_unique<Misc::Barrier>(mNumThreads);

        mPostStepBarrier = std::make_unique<Misc::Barrier>(mNumThreads);

        mPostSimBarrier = std::make_unique<Misc::Barrier>(mNumThreads);
    }

    PhysicsTaskScheduler::~PhysicsTaskScheduler()
    {
        waitForWorkers();
        {
            MaybeExclusiveLock lock(mSimulationMutex, mLockingPolicy);
            mNumJobs = 0;
            mRemainingSteps = 0;
        }
        if (mWorkersSync != nullptr)
            mWorkersSync->stopWorkers();
        for (auto& thread : mThreads)
            thread.join();
    }

    std::tuple<unsigned, float> PhysicsTaskScheduler::calculateStepConfig(float timeAccum) const
    {
        unsigned maxAllowedSteps = 2;
        unsigned numSteps = static_cast<unsigned>(timeAccum / mDefaultPhysicsDt);

        // adjust maximum step count based on whether we're likely physics bottlenecked or not
        // if maxAllowedSteps ends up higher than numSteps, we will not invoke delta time
        // if it ends up lower than numSteps, but greater than 1, we will run a number of true delta time physics steps
        // that we expect to be within budget if it ends up lower than numSteps and also 1, we will run a single delta
        // time physics step if we did not do this, and had a fixed step count limit, we would have an unnecessarily low
        // render framerate if we were only physics bottlenecked, and we would be unnecessarily invoking true delta time
        // if we were only render bottlenecked

        // get physics timing stats
        float budgetMeasurement = std::max(mBudget.get(), mAsyncBudget.get());
        // time spent per step in terms of the intended physics framerate
        budgetMeasurement /= mDefaultPhysicsDt;
        // ensure sane minimum value
        budgetMeasurement = std::max(0.00001f, budgetMeasurement);
        // we're spending almost or more than realtime per physics frame; limit to a single step
        if (budgetMeasurement > 0.95f)
            maxAllowedSteps = 1;
        // physics is fairly cheap; limit based on expense
        if (budgetMeasurement < 0.5f)
            maxAllowedSteps = static_cast<unsigned>(std::ceil(1.f / budgetMeasurement));
        // limit to a reasonable amount
        maxAllowedSteps = std::min(10u, maxAllowedSteps);

        // fall back to delta time for this frame if fixed timestep physics would fall behind
        float actualDelta = mDefaultPhysicsDt;
        if (numSteps > maxAllowedSteps)
        {
            numSteps = maxAllowedSteps;
            // ensure that we do not simulate a frame ahead when doing delta time; this reduces stutter and latency
            // this causes interpolation to 100% use the most recent physics result when true delta time is happening
            // and we deliberately simulate up to exactly the timestamp that we want to render
            actualDelta = timeAccum / float(numSteps + 1);
            // actually: if this results in a per-step delta less than the target physics steptime, clamp it
            // this might reintroduce some stutter, but only comes into play in obscure cases
            // (because numSteps is originally based on mDefaultPhysicsDt, this won't cause us to overrun)
            actualDelta = std::max(actualDelta, mDefaultPhysicsDt);
        }

        return std::make_tuple(numSteps, actualDelta);
    }

    void PhysicsTaskScheduler::applyQueuedMovements(float& timeAccum, std::vector<Simulation>& simulations,
        osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats)
    {
        assert(mSimulations != &simulations);

        waitForWorkers();
        prepareWork(timeAccum, simulations, frameStart, frameNumber, stats);
        if (mWorkersSync != nullptr)
            mWorkersSync->wakeUpWorkers();
    }

    void PhysicsTaskScheduler::prepareWork(float& timeAccum, std::vector<Simulation>& simulations,
        osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats)
    {
        // This function run in the main thread.
        // While the mSimulationMutex is held, background physics threads can't run.

        MaybeExclusiveLock lock(mSimulationMutex, mLockingPolicy);

        auto timeStart = mTimer->tick();

        // start by finishing previous background computation
        if (mNumThreads != 0)
        {
            syncWithMainThread();

            if (mAdvanceSimulation)
                mAsyncBudget.update(mTimer->delta_s(mAsyncStartTime, mTimeEnd), mPrevStepCount, mBudgetCursor);
            updateStats(frameStart, frameNumber, stats);
        }

        auto [numSteps, newDelta] = calculateStepConfig(timeAccum);
        timeAccum -= numSteps * newDelta;

        // init
        const Visitors::InitPosition vis{ mCollisionWorld };
        for (auto& sim : simulations)
        {
            std::visit(vis, sim);
        }
        mPrevStepCount = numSteps;
        mRemainingSteps = numSteps;
        mTimeAccum = timeAccum;
        mPhysicsDt = newDelta;
        mSimulations = &simulations;
        mAdvanceSimulation = (mRemainingSteps != 0);
        mNumJobs = static_cast<int>(mSimulations->size());
        mNextLOS.store(0, std::memory_order_relaxed);
        mNextJob.store(0, std::memory_order_release);

        if (mAdvanceSimulation)
            mWorldFrameData = std::make_unique<WorldFrameData>();

        if (mAdvanceSimulation)
            mBudgetCursor += 1;

        if (mNumThreads == 0)
        {
            doSimulation();
            syncWithMainThread();
            if (mAdvanceSimulation)
                mBudget.update(mTimer->delta_s(timeStart, mTimer->tick()), numSteps, mBudgetCursor);
            return;
        }

        mAsyncStartTime = mTimer->tick();
        if (mAdvanceSimulation)
            mBudget.update(mTimer->delta_s(timeStart, mTimer->tick()), 1, mBudgetCursor);
    }

    void PhysicsTaskScheduler::resetSimulation(const ActorMap& actors)
    {
        waitForWorkers();
        MaybeExclusiveLock lock(mSimulationMutex, mLockingPolicy);
        mBudget.reset(mDefaultPhysicsDt);
        mAsyncBudget.reset(0.0f);
        if (mSimulations != nullptr)
        {
            mSimulations->clear();
            mSimulations = nullptr;
        }
        for (const auto& [_, actor] : actors)
        {
            actor->updatePosition();
            actor->updateCollisionObjectPosition();
        }
    }

    void PhysicsTaskScheduler::rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld,
        btCollisionWorld::RayResultCallback& resultCallback) const
    {
        MaybeLock lock(mCollisionWorldMutex, mLockingPolicy);
        mCollisionWorld->rayTest(rayFromWorld, rayToWorld, resultCallback);
    }

    void PhysicsTaskScheduler::convexSweepTest(const btConvexShape* castShape, const btTransform& from,
        const btTransform& to, btCollisionWorld::ConvexResultCallback& resultCallback) const
    {
        MaybeLock lock(mCollisionWorldMutex, mLockingPolicy);
        mCollisionWorld->convexSweepTest(castShape, from, to, resultCallback);
    }

    void PhysicsTaskScheduler::contactTest(
        btCollisionObject* colObj, btCollisionWorld::ContactResultCallback& resultCallback)
    {
        MaybeSharedLock lock(mCollisionWorldMutex, mLockingPolicy);
        ContactTestWrapper::contactTest(mCollisionWorld, colObj, resultCallback);
    }

    std::optional<btVector3> PhysicsTaskScheduler::getHitPoint(const btTransform& from, btCollisionObject* target)
    {
        MaybeLock lock(mCollisionWorldMutex, mLockingPolicy);
        // target the collision object's world origin, this should be the center of the collision object
        btTransform rayTo;
        rayTo.setIdentity();
        rayTo.setOrigin(target->getWorldTransform().getOrigin());

        btCollisionWorld::ClosestRayResultCallback cb(from.getOrigin(), rayTo.getOrigin());

        mCollisionWorld->rayTestSingle(
            from, rayTo, target, target->getCollisionShape(), target->getWorldTransform(), cb);
        if (!cb.hasHit())
            // didn't hit the target. this could happen if point is already inside the collision box
            return std::nullopt;
        return { cb.m_hitPointWorld };
    }

    void PhysicsTaskScheduler::aabbTest(
        const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback)
    {
        MaybeSharedLock lock(mCollisionWorldMutex, mLockingPolicy);
        mCollisionWorld->getBroadphase()->aabbTest(aabbMin, aabbMax, callback);
    }

    void PhysicsTaskScheduler::getAabb(const btCollisionObject* obj, btVector3& min, btVector3& max)
    {
        MaybeSharedLock lock(mCollisionWorldMutex, mLockingPolicy);
        obj->getCollisionShape()->getAabb(obj->getWorldTransform(), min, max);
    }

    void PhysicsTaskScheduler::setCollisionFilterMask(btCollisionObject* collisionObject, int collisionFilterMask)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        collisionObject->getBroadphaseHandle()->m_collisionFilterMask = collisionFilterMask;
    }

    void PhysicsTaskScheduler::addCollisionObject(
        btCollisionObject* collisionObject, int collisionFilterGroup, int collisionFilterMask)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        mCollisionObjects.insert(collisionObject);
        mCollisionWorld->addCollisionObject(collisionObject, collisionFilterGroup, collisionFilterMask);
    }

    void PhysicsTaskScheduler::addRigidBody(btRigidBody* body, int collisionFilterGroup, int collisionFilterMask)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        mCollisionObjects.insert(body);
        mDynamicsWorld->addRigidBody(body, collisionFilterGroup, collisionFilterMask);
        ++mNumRigidBodies;
    }

    void PhysicsTaskScheduler::addConstraint(btTypedConstraint* constraint)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        mDynamicsWorld->addConstraint(constraint, true);
    }

    void PhysicsTaskScheduler::removeConstraint(btTypedConstraint* constraint)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        mDynamicsWorld->removeConstraint(constraint);
    }

    void PhysicsTaskScheduler::removeCollisionObject(btCollisionObject* collisionObject)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        mCollisionObjects.erase(collisionObject);
        if (btRigidBody::upcast(collisionObject) != nullptr)
            --mNumRigidBodies;
        mCollisionWorld->removeCollisionObject(collisionObject);
    }

    void PhysicsTaskScheduler::updateSingleAabb(const std::shared_ptr<PtrHolder>& ptr, bool immediate)
    {
        if (immediate || mNumThreads == 0)
        {
            updatePtrAabb(ptr);
        }
        else
        {
            MaybeExclusiveLock lock(mUpdateAabbMutex, mLockingPolicy);
            mUpdateAabb.insert(ptr);
        }
    }

    bool PhysicsTaskScheduler::getLineOfSight(
        const std::shared_ptr<Actor>& actor1, const std::shared_ptr<Actor>& actor2)
    {
        MaybeExclusiveLock lock(mLOSCacheMutex, mLockingPolicy);

        auto req = LOSRequest(actor1, actor2);
        auto result = std::find(mLOSCache.begin(), mLOSCache.end(), req);
        if (result == mLOSCache.end())
        {
            req.mResult = hasLineOfSight(actor1.get(), actor2.get());
            mLOSCache.push_back(req);
            return req.mResult;
        }
        result->mAge = 0;
        return result->mResult;
    }

    void PhysicsTaskScheduler::refreshLOSCache()
    {
        MaybeSharedLock lock(mLOSCacheMutex, mLockingPolicy);
        int job = 0;
        int numLOS = static_cast<int>(mLOSCache.size());
        while ((job = mNextLOS.fetch_add(1, std::memory_order_relaxed)) < numLOS)
        {
            auto& req = mLOSCache[job];
            auto actorPtr1 = req.mActors[0].lock();
            auto actorPtr2 = req.mActors[1].lock();

            if (req.mAge++ > mLOSCacheExpiry || !actorPtr1 || !actorPtr2)
                req.mStale = true;
            else
                req.mResult = hasLineOfSight(actorPtr1.get(), actorPtr2.get());
        }
    }

    void PhysicsTaskScheduler::updateAabbs()
    {
        MaybeExclusiveLock lock(mUpdateAabbMutex, mLockingPolicy);
        std::for_each(mUpdateAabb.begin(), mUpdateAabb.end(), [this](const std::weak_ptr<PtrHolder>& ptr) {
            auto p = ptr.lock();
            if (p != nullptr)
                updatePtrAabb(p);
        });
        mUpdateAabb.clear();
    }

    void PhysicsTaskScheduler::updatePtrAabb(const std::shared_ptr<PtrHolder>& ptr)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        if (const auto actor = std::dynamic_pointer_cast<Actor>(ptr))
        {
            actor->updateCollisionObjectPosition();
            mCollisionWorld->updateSingleAabb(actor->getCollisionObject());
        }
        else if (const auto object = std::dynamic_pointer_cast<Object>(ptr))
        {
            object->commitPositionChange();
            if (const std::optional<bool> stuck = object->takePendingStuck())
                setStuckUnsafe(*object, *stuck);
            mCollisionWorld->updateSingleAabb(object->getCollisionObject());
            if (btCollisionObject* detail = object->getDetailCollisionObject())
                mCollisionWorld->updateSingleAabb(detail);
        }
        else if (const auto projectile = std::dynamic_pointer_cast<Projectile>(ptr))
        {
            projectile->updateCollisionObjectPosition();
            mCollisionWorld->updateSingleAabb(projectile->getCollisionObject());
        }
    }

    void PhysicsTaskScheduler::worker()
    {
        mWorkersSync->runWorker([this] {
            std::shared_lock lock(mSimulationMutex);
            doSimulation();
        });
    }

    void PhysicsTaskScheduler::updateActorsPositions()
    {
        const Visitors::UpdatePosition impl{ mCollisionWorld };
        const Visitors::WithLockedPtr<Visitors::UpdatePosition, MaybeExclusiveLock> vis{ impl, mCollisionWorldMutex,
            mLockingPolicy };
        for (Simulation& sim : *mSimulations)
            std::visit(vis, sim);
    }

    bool PhysicsTaskScheduler::hasLineOfSight(const Actor* actor1, const Actor* actor2)
    {
        btVector3 pos1 = Misc::Convert::toBullet(
            actor1->getCollisionObjectPosition() + osg::Vec3f(0, 0, actor1->getHalfExtents().z() * 0.9f)); // eye level
        btVector3 pos2 = Misc::Convert::toBullet(
            actor2->getCollisionObjectPosition() + osg::Vec3f(0, 0, actor2->getHalfExtents().z() * 0.9f));

        if ((pos2 - pos1).fuzzyZero())
            return true;

        btCollisionWorld::ClosestRayResultCallback resultCallback(pos1, pos2);
        resultCallback.m_collisionFilterGroup = CollisionType_AnyPhysical;
        resultCallback.m_collisionFilterMask = CollisionType_World | CollisionType_HeightMap | CollisionType_Door;

        MaybeLock lockColWorld(mCollisionWorldMutex, mLockingPolicy);
        mCollisionWorld->rayTest(pos1, pos2, resultCallback);

        return !resultCallback.hasHit();
    }

    void PhysicsTaskScheduler::doSimulation()
    {
        while (mRemainingSteps)
        {
            mPreStepBarrier->wait([this] { afterPreStep(); });
            int job = 0;
            const Visitors::Move impl{ mPhysicsDt, mCollisionWorld, *mWorldFrameData };
            const Visitors::WithLockedPtr<Visitors::Move, MaybeLock> vis{ impl, mCollisionWorldMutex, mLockingPolicy };
            while ((job = mNextJob.fetch_add(1, std::memory_order_relaxed)) < mNumJobs)
                std::visit(vis, (*mSimulations)[job]);

            mPostStepBarrier->wait([this] { afterPostStep(); });
        }

        refreshLOSCache();
        mPostSimBarrier->wait([this] { afterPostSim(); });
    }

    void PhysicsTaskScheduler::updateStats(osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats)
    {
        if (!stats.collectStats("engine"))
            return;
        if (mFrameNumber == frameNumber - 1)
        {
            stats.setAttribute(mFrameNumber, "physicsworker_time_begin", mTimer->delta_s(mFrameStart, mTimeBegin));
            stats.setAttribute(mFrameNumber, "physicsworker_time_taken", mTimer->delta_s(mTimeBegin, mTimeEnd));
            stats.setAttribute(mFrameNumber, "physicsworker_time_end", mTimer->delta_s(mFrameStart, mTimeEnd));
        }
        mFrameStart = frameStart;
        mTimeBegin = mTimer->tick();
        mFrameNumber = frameNumber;
    }

    void PhysicsTaskScheduler::debugDraw()
    {
        MaybeSharedLock lock(mCollisionWorldMutex, mLockingPolicy);
        mDebugDrawer->step();
    }

    void* PhysicsTaskScheduler::getUserPointer(const btCollisionObject* object) const
    {
        auto it = mCollisionObjects.find(object);
        if (it == mCollisionObjects.end())
            return nullptr;
        return (*it)->getUserPointer();
    }

    void PhysicsTaskScheduler::releaseSharedStates()
    {
        waitForWorkers();
        std::scoped_lock lock(mSimulationMutex, mUpdateAabbMutex);
        if (mSimulations != nullptr)
        {
            mSimulations->clear();
            mSimulations = nullptr;
        }
        mUpdateAabb.clear();
    }

    void PhysicsTaskScheduler::afterPreStep()
    {
        updateAabbs();
        if (!mRemainingSteps)
            return;
        const Visitors::PreStep impl{ mCollisionWorld };
        const Visitors::WithLockedPtr<Visitors::PreStep, MaybeExclusiveLock> vis{ impl, mCollisionWorldMutex,
            mLockingPolicy };
        for (auto& sim : *mSimulations)
            std::visit(vis, sim);
    }

    void PhysicsTaskScheduler::afterPostStep()
    {
        if (mRemainingSteps)
        {
            --mRemainingSteps;
            updateActorsPositions();
            pushDynamicObjects();
            stepDynamics();
        }
        mNextJob.store(0, std::memory_order_release);
    }

    void PhysicsTaskScheduler::pushDynamicObjects()
    {
        if (mNumRigidBodies == 0)
            return;

        // Locked actors must outlive the collision world lock (see WithLockedPtr).
        std::vector<std::shared_ptr<Actor>> actors;
        for (auto& sim : *mSimulations)
            if (auto* actorSim = std::get_if<ActorSimulation>(&sim))
                if (auto locked = actorSim->lock())
                    actors.push_back(std::move(locked->first));
        const std::shared_ptr<PtrHolder> held = getHeldObject();
        // Of a held ragdoll, all of it (the carrier would trip over the body it drags).
        const MWWorld::LiveCellRefBase* const heldRef = held != nullptr ? held->getPtr().mRef : nullptr;

        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        for (const auto& actor : actors)
        {
            const osg::Vec3f displacement = actor->getPosition() - actor->getPreviousPosition();
            btVector3 actorVelocity = Misc::Convert::toBullet(displacement) / mPhysicsDt;
            if (actorVelocity.length2() < 1.f)
                continue;
            // Teleports also show up as displacement; don't fling things across the room.
            constexpr btScalar maxPushSpeed = 1000.f;
            if (actorVelocity.length2() > maxPushSpeed * maxPushSpeed)
                actorVelocity *= maxPushSpeed / actorVelocity.length();

            // Actors walk through dynamic objects (their movement ignores them); any object the actor now
            // overlaps gets shoved so the point of contact keeps up with the actor.
            DynamicContactCallback callback(actor->getCollisionObject());
            ContactTestWrapper::contactTest(mCollisionWorld, actor->getCollisionObject(), callback);
            for (const auto& contact : callback.mContacts)
            {
                // A carried object is steered by the carrier, not shoved; a stuck one doesn't budge.
                if ((held != nullptr && contact.mBody == held->getCollisionObject()) || contact.mBody->getInvMass() == 0)
                    continue;
                if (const auto* holder = static_cast<const PtrHolder*>(contact.mBody->getUserPointer());
                    heldRef != nullptr && holder != nullptr && holder->getPtr().mRef == heldRef)
                    continue;

                // Push horizontally, away from the actor. If the contact is (nearly) vertical, e.g. the actor is
                // stepping over the object, push it along the actor's direction of movement instead.
                btVector3 direction = contact.mPushDirection;
                direction.setZ(0);
                if (direction.length2() < 0.25f)
                    direction = btVector3(actorVelocity.x(), actorVelocity.y(), 0);
                if (direction.length2() < 1e-4f)
                    continue;
                direction.normalize();

                const btScalar approach = actorVelocity.dot(direction);
                if (approach <= 0)
                    continue;
                const btVector3 relativePoint = contact.mPoint - contact.mBody->getCenterOfMassPosition();
                const btScalar deficit = approach - contact.mBody->getVelocityInLocalPoint(relativePoint).dot(direction);
                if (deficit <= 0)
                    continue;

                contact.mBody->activate(true);
                contact.mBody->applyImpulse(direction * (deficit / contact.mBody->getInvMass()), relativePoint);            }
        }
    }

    void PhysicsTaskScheduler::stepDynamics()
    {
        if (mNumRigidBodies == 0)
            return;

        // Locked object must outlive the collision world lock (its destructor takes the lock).
        std::shared_ptr<PtrHolder> held;
        btVector3 holdTarget;
        btQuaternion holdTargetRotation;
        bool steerRotation;
        {
            std::lock_guard heldLock(mHeldObjectMutex);
            held = mHeldObject.lock();
            holdTarget = mHoldTarget;
            holdTargetRotation = mHoldTargetRotation;
            steerRotation = mHoldSteerRotation;
        }
        std::vector<std::shared_ptr<Object>> wedged;
        {
            std::lock_guard wedgedLock(mWedgedObjectsMutex);
            for (const std::weak_ptr<Object>& object : mWedgedObjects)
                if (std::shared_ptr<Object> locked = object.lock())
                    wedged.push_back(std::move(locked));
        }
        std::vector<std::pair<std::shared_ptr<Object>, FlyingObject>> flying;
        {
            std::lock_guard flyingLock(mFlyingObjectsMutex);
            for (const FlyingObject& object : mFlyingObjects)
                if (std::shared_ptr<Object> locked = object.mObject.lock())
                    flying.emplace_back(std::move(locked), object);
        }

        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);

        updateWedgedObjects(wedged);

        if (held != nullptr && btRigidBody::upcast(held->getCollisionObject()) != nullptr)
        {
            btRigidBody& body = *btRigidBody::upcast(held->getCollisionObject());
            const btVector3 offset = holdTarget - body.getCenterOfMassPosition();
            // Stuck behind something, or the carrier moved away too fast: let go.
            constexpr btScalar maxHoldDistance = 200.f;
            if (offset.length2() > maxHoldDistance * maxHoldDistance)
            {
                releaseHeldObjectUnsafe(body, std::nullopt);
                std::lock_guard heldLock(mHeldObjectMutex);
                mHeldObject.reset();
            }
            else
            {
                // Close most of the remaining gap each step; collisions still stop the object. Something jointed
                // (a ragdoll limb, not steered to an orientation) follows more loosely, so it doesn't fight its
                // joints or jerk with every bob of the carrier's head.
                btVector3 velocity = offset * ((steerRotation ? 0.5f : 0.12f) / mPhysicsDt);
                const btScalar maxCarrySpeed = steerRotation ? 2000.f : 500.f;
                if (velocity.length2() > maxCarrySpeed * maxCarrySpeed)
                    velocity *= maxCarrySpeed / velocity.length();
                body.setLinearVelocity(velocity);
            }
            if (offset.length2() <= maxHoldDistance * maxHoldDistance && steerRotation)
            {
                // Likewise turn it towards the target orientation, the short way round.
                btQuaternion error = holdTargetRotation * body.getOrientation().inverse();
                if (error.getW() < 0)
                    error = -error;
                const btScalar angle = error.getAngle();
                btVector3 angularVelocity(0, 0, 0);
                if (angle > 1e-4f)
                    angularVelocity = error.getAxis() * (angle * 0.5f / mPhysicsDt);
                constexpr btScalar maxCarrySpin = 20.f; // rad/s
                if (angularVelocity.length2() > maxCarrySpin * maxCarrySpin)
                    angularVelocity *= maxCarrySpin / angularVelocity.length();
                body.setAngularVelocity(angularVelocity);
            }
        }

        if (mHasWater)
            applyWaterForces(mWaterHeight);

        // We are already called once per fixed physics step, so no substepping.
        mDynamicsWorld->stepSimulation(mPhysicsDt, 0);

        updateFlyingObjects(flying);
    }

    void PhysicsTaskScheduler::setWaterHeight(std::optional<float> height)
    {
        mWaterHeight = height.value_or(0.f);
        mHasWater = height.has_value();
    }

    void PhysicsTaskScheduler::applyWaterForces(float waterHeight)
    {
        // Called with the collision world locked.
        const btVector3 gravity = mDynamicsWorld->getGravity();
        const btCollisionObjectArray& objects = mDynamicsWorld->getCollisionObjectArray();
        for (int i = 0; i < objects.size(); ++i)
        {
            btRigidBody* body = btRigidBody::upcast(objects[i]);
            if (body == nullptr || body->isStaticOrKinematicObject() || !body->isActive()
                || body->getInvMass() == 0)
                continue;
            btVector3 aabbMin;
            btVector3 aabbMax;
            body->getAabb(aabbMin, aabbMax);
            if (aabbMin.z() >= waterHeight)
                continue;

            // Buoyancy: the weight of the displaced water. Lighter than water floats, sitting at the depth where
            // the two balance; heavier sinks, more slowly. It acts on sample points spread through the object,
            // each by how deep it is, so the deeper side gets pushed up harder and the object turns into a
            // natural floating position (a bottle on its side, a book flat) instead of keeping whatever angle it
            // landed at.
            // Items know how dense they are; anything else (a body) is about as dense as water.
            const auto* object = dynamic_cast<const Object*>(static_cast<const PtrHolder*>(body->getUserPointer()));
            const float relativeDensity = object != nullptr ? object->getRelativeDensity() : 1.05f;
            const btScalar mass = 1 / body->getInvMass();
            btVector3 localMin;
            btVector3 localMax;
            body->getCollisionShape()->getAabb(btTransform::getIdentity(), localMin, localMax);
            const btVector3 halfExtents = (localMax - localMin) * 0.5;
            // How much of the depth range a sample point stands for.
            const btScalar pointRadius = std::max(halfExtents[halfExtents.minAxis()], btScalar(1));
            const btTransform& transform = body->getWorldTransform();
            constexpr int numPoints = 9; // the center and the eight corners, pulled in a bit
            const btVector3 pointBuoyancy = -gravity * (mass / relativeDensity / numPoints);
            btScalar submerged = 0;
            for (int p = 0; p < numPoints; ++p)
            {
                btVector3 local(0, 0, 0);
                if (p > 0)
                    local = btVector3((p & 1) ? 0.7f : -0.7f, (p & 2) ? 0.7f : -0.7f, (p & 4) ? 0.7f : -0.7f)
                        * halfExtents;
                const btVector3 point = transform * local;
                const btScalar pointSubmerged = std::clamp(
                    (waterHeight - point.z()) / (2 * pointRadius) + btScalar(0.5), btScalar(0), btScalar(1));
                if (pointSubmerged <= 0)
                    continue;
                body->applyForce(pointBuoyancy * pointSubmerged, point - transform.getOrigin());
                submerged += pointSubmerged / numPoints;
            }

            // Water drag, so sinking things drift down and floating things settle rather than bob forever. It
            // slows light things more (less mass behind the same surface) and fast things more (drag grows with
            // speed), so whatever hits the water hard is caught by it instead of skipping off the surface.
            constexpr btScalar waterDrag = 4.f; // per second, fully submerged, at rest, for water-dense things
            const btScalar speed = body->getLinearVelocity().length();
            const btScalar drag = waterDrag * std::max(btScalar(1), btScalar(1 / relativeDensity))
                * (1 + speed / 400.f);
            const btScalar damping = std::max(btScalar(0.3), 1 - drag * submerged * mPhysicsDt);
            body->setLinearVelocity(body->getLinearVelocity() * damping);
            // Gentler on spin, so it can still turn to float naturally.
            constexpr btScalar waterSpinDrag = 2.f; // per second, fully submerged
            body->setAngularVelocity(
                body->getAngularVelocity() * std::max(btScalar(0.5), 1 - waterSpinDrag * submerged * mPhysicsDt));
        }
    }

    void PhysicsTaskScheduler::addFlyingObject(const std::shared_ptr<Object>& object, const btCollisionObject* thrower)
    {
        std::lock_guard flyingLock(mFlyingObjectsMutex);
        mFlyingObjects.push_back({ object, thrower });
    }

    std::vector<PhysicsTaskScheduler::FlyingObjectHit> PhysicsTaskScheduler::takeFlyingObjectHits()
    {
        std::lock_guard flyingLock(mFlyingObjectsMutex);
        return std::exchange(mFlyingObjectHits, {});
    }

    void PhysicsTaskScheduler::updateFlyingObjects(
        const std::vector<std::pair<std::shared_ptr<Object>, FlyingObject>>& objects)
    {
        // Called with the collision world locked; the caller keeps the objects alive until it is unlocked.
        if (objects.empty())
            return;

        std::vector<const Object*> landed;
        std::vector<FlyingObjectHit> hits;
        for (const auto& [object, flying] : objects)
        {
            btRigidBody& body = *object->getRigidBody();
            const btScalar speed = body.getLinearVelocity().length();
            // Once it has slowed down it is just lying or rolling around, not a projectile any more.
            constexpr btScalar minHitSpeed = 150.f;
            if (speed < minHitSpeed)
            {
                landed.push_back(object.get());
                continue;
            }

            ActorHitCallback actorHit(&body, flying.mThrower);
            ContactTestWrapper::contactTest(mCollisionWorld, &body, actorHit);
            if (actorHit.mActor == nullptr)
                continue;

            // Actors aren't solid to simulated objects, so bounce it off by hand.
            const btVector3 velocity = body.getLinearVelocity();
            const btScalar into = velocity.dot(actorHit.mNormal);
            if (into < 0)
                body.setLinearVelocity((velocity - actorHit.mNormal * (into * 1.4f)) * 0.6f);
            hits.push_back({ object, actorHit.mActor, speed });
            landed.push_back(object.get()); // one hit per throw
        }

        std::lock_guard flyingLock(mFlyingObjectsMutex);
        // Note: no weak_ptr::lock() while the collision world is locked (see stepDynamics).
        std::erase_if(mFlyingObjects, [&](const FlyingObject& flying) {
            if (flying.mObject.expired())
                return true;
            return std::any_of(landed.begin(), landed.end(), [&](const Object* object) {
                const auto found = std::find_if(objects.begin(), objects.end(),
                    [&](const auto& pair) { return pair.first.get() == object; });
                return !flying.mObject.owner_before(found->first) && !found->first.owner_before(flying.mObject);
            });
        });
        mFlyingObjectHits.insert(mFlyingObjectHits.end(), hits.begin(), hits.end());
    }

    void PhysicsTaskScheduler::holdObject(
        const std::shared_ptr<PtrHolder>& holder, bool steerRotation, btScalar holdMass)
    {
        releaseHeldObject(std::nullopt);

        btRigidBody* const heldBody = btRigidBody::upcast(holder->getCollisionObject());
        if (heldBody == nullptr)
            return;
        btRigidBody& body = *heldBody;
        const std::shared_ptr<Object> object = std::dynamic_pointer_cast<Object>(holder);

        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        // Grabbing is how stuck things (arrows in a wall) come loose.
        if (object != nullptr)
            setStuckUnsafe(*object, false);
        body.setActivationState(DISABLE_DEACTIVATION);
        body.setGravity(btVector3(0, 0, 0));
        if (object != nullptr)
            freeWedgedObjectUnsafe(object);
        mHeldRestoreMass = 0; // left over if the last held thing was destroyed while held
        if (holdMass > 0 && body.getInvMass() > 0)
        {
            mHeldRestoreMass = 1 / body.getInvMass();
            btVector3 inertia(0, 0, 0);
            body.getCollisionShape()->calculateLocalInertia(holdMass, inertia);
            body.setMassProps(holdMass, inertia);
            body.updateInertiaTensor();
        }

        std::lock_guard heldLock(mHeldObjectMutex);
        mHeldObject = holder;
        mHoldSteerRotation = steerRotation;
        mHoldTarget = body.getCenterOfMassPosition();
        mHoldTargetRotation = body.getOrientation();
    }

    void PhysicsTaskScheduler::setHoldTarget(const btVector3& position, const btQuaternion& rotation)
    {
        std::lock_guard heldLock(mHeldObjectMutex);
        mHoldTarget = position;
        mHoldTargetRotation = rotation;
    }

    void PhysicsTaskScheduler::releaseHeldObject(const std::optional<btVector3>& velocity)
    {
        const std::shared_ptr<PtrHolder> held = getHeldObject();
        if (held == nullptr)
            return;
        if (btRigidBody* body = btRigidBody::upcast(held->getCollisionObject()))
        {
            MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
            releaseHeldObjectUnsafe(*body, velocity);
        }
        std::lock_guard heldLock(mHeldObjectMutex);
        mHeldObject.reset();
    }

    void PhysicsTaskScheduler::releaseHeldObjectUnsafe(btRigidBody& body, const std::optional<btVector3>& velocity)
    {
        if (mHeldRestoreMass > 0)
        {
            btVector3 inertia(0, 0, 0);
            body.getCollisionShape()->calculateLocalInertia(mHeldRestoreMass, inertia);
            body.setMassProps(mHeldRestoreMass, inertia);
            body.updateInertiaTensor();
            mHeldRestoreMass = 0;
        }
        body.forceActivationState(ACTIVE_TAG);
        body.setDeactivationTime(0);
        body.setGravity(mDynamicsWorld->getGravity());
        if (velocity)
            body.setLinearVelocity(*velocity);
        else
        {
            // Dropped: don't keep the speed it had while being steered around.
            constexpr btScalar maxDropSpeed = 300.f;
            const btVector3 current = body.getLinearVelocity();
            if (current.length2() > maxDropSpeed * maxDropSpeed)
                body.setLinearVelocity(current * (maxDropSpeed / current.length()));
        }
    }

    void PhysicsTaskScheduler::setStuckUnsafe(Object& object, bool stuck)
    {
        // Called with the collision world locked. A stuck object is static (no mass): nothing moves it, things
        // bounce off it. Changing that means taking it out of the world and putting it back.
        if (object.isStuck() == stuck || object.getRigidBody() == nullptr)
            return;
        btRigidBody& body = *object.getRigidBody();
        const int group = body.getBroadphaseHandle()->m_collisionFilterGroup;
        const int mask = body.getBroadphaseHandle()->m_collisionFilterMask;
        mDynamicsWorld->removeRigidBody(&body);
        body.setLinearVelocity(btVector3(0, 0, 0));
        body.setAngularVelocity(btVector3(0, 0, 0));
        if (stuck)
            body.setMassProps(0, btVector3(0, 0, 0));
        else
        {
            body.setMassProps(object.getMass(), object.getLocalInertia());
            body.updateInertiaTensor();
        }
        mDynamicsWorld->addRigidBody(&body, group, mask);
        if (!stuck)
        {
            body.setGravity(mDynamicsWorld->getGravity());
            body.forceActivationState(ACTIVE_TAG);
        }
        object.setStuckFlag(stuck);
    }

    void PhysicsTaskScheduler::freeWedgedObjectUnsafe(const std::shared_ptr<Object>& object)
    {
        // Placed items often sit partly inside what they rest on (books in shelves), which wedges them in place
        // once they are grabbed or struck. Let the item pass through whatever it is stuck in until it is clear.
        btRigidBody& body = *object->getRigidBody();
        PenetrationCallback penetrating(&body, 0.5f);
        ContactTestWrapper::contactTest(mCollisionWorld, &body, penetrating);
        std::vector<const btCollisionObject*>& wedgedIn = object->getWedgedIn();
        for (const btCollisionObject* other : penetrating.mOthers)
        {
            if (std::find(wedgedIn.begin(), wedgedIn.end(), other) != wedgedIn.end())
                continue;
            body.setIgnoreCollisionCheck(other, true);
            wedgedIn.push_back(other);
        }
        if (wedgedIn.empty())
            return;

        // Note: no weak_ptr::lock() while the collision world is locked (see stepDynamics).
        std::lock_guard wedgedLock(mWedgedObjectsMutex);
        const bool tracked
            = std::any_of(mWedgedObjects.begin(), mWedgedObjects.end(), [&](const std::weak_ptr<Object>& tracked) {
                  return !tracked.owner_before(object) && !object.owner_before(tracked);
              });
        if (!tracked)
            mWedgedObjects.push_back(object);
    }

    void PhysicsTaskScheduler::updateWedgedObjects(const std::vector<std::shared_ptr<Object>>& objects)
    {
        // Called with the collision world locked; the caller keeps the objects alive until it is unlocked.
        if (objects.empty())
            return;
        for (const std::shared_ptr<Object>& object : objects)
        {
            btRigidBody& body = *object->getRigidBody();
            std::erase_if(object->getWedgedIn(), [&](const btCollisionObject* other) {
                // Static objects can be unloaded in the meantime; only touch ones still in the world.
                if (mCollisionObjects.find(other) == mCollisionObjects.end())
                    return true;
                PenetrationCallback overlap(&body, 0.f);
                ContactTestWrapper::contactPairTest(
                    mCollisionWorld, &body, const_cast<btCollisionObject*>(other), overlap);
                if (!overlap.mOthers.empty())
                    return false;
                body.setIgnoreCollisionCheck(other, false);
                return true;
            });
        }

        // Note: no weak_ptr::lock() while the collision world is locked (see stepDynamics).
        std::lock_guard wedgedLock(mWedgedObjectsMutex);
        std::erase_if(mWedgedObjects, [&](const std::weak_ptr<Object>& tracked) {
            if (tracked.expired())
                return true;
            const auto found = std::find_if(objects.begin(), objects.end(), [&](const std::shared_ptr<Object>& o) {
                return !tracked.owner_before(o) && !o.owner_before(tracked);
            });
            return found != objects.end() && (*found)->getWedgedIn().empty();
        });
    }

    void PhysicsTaskScheduler::strikeObjects(const std::vector<Strike>& strikes)
    {
        // The strikes hold the objects, so none can be destroyed while the collision world is locked.
        const std::shared_ptr<PtrHolder> held = getHeldObject();
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        for (const Strike& strike : strikes)
        {
            // Stuck objects only come loose when grabbed.
            if (strike.mObject == held || strike.mObject->isStuck())
                continue;
            btRigidBody& body = *strike.mObject->getRigidBody();
            body.activate(true);
            body.applyImpulse(
                strike.mVelocityChange / body.getInvMass(), strike.mPoint - body.getCenterOfMassPosition());
            // Knock it out of the shelf it is wedged in, like grabbing it would.
            freeWedgedObjectUnsafe(strike.mObject);

            // Things sitting in furniture (books in a bookcase) would mostly be driven into it and stay. Pop them
            // out towards the side the blow came from, which for shelves is the open front.
            FurnitureContactCallback furniture(&body);
            ContactTestWrapper::contactTest(mCollisionWorld, &body, furniture);
            if (furniture.mFurniture != nullptr)
            {
                btVector3 aabbMin;
                btVector3 aabbMax;
                furniture.mFurniture->getCollisionShape()->getAabb(
                    furniture.mFurniture->getWorldTransform(), aabbMin, aabbMax);
                btVector3 out = strike.mSource - (aabbMin + aabbMax) * 0.5;
                out.setZ(0);
                if (out.length2() > 1e-4f)
                {
                    out.normalize();
                    constexpr btScalar popOutSpeed = 500.f;
                    constexpr btScalar popUpSpeed = 200.f;
                    body.applyCentralImpulse(
                        (out * popOutSpeed + btVector3(0, 0, popUpSpeed)) / body.getInvMass());
                }
            }
        }
    }

    std::shared_ptr<PtrHolder> PhysicsTaskScheduler::getHeldObject() const
    {
        std::lock_guard heldLock(mHeldObjectMutex);
        return mHeldObject.lock();
    }

    void PhysicsTaskScheduler::afterPostSim()
    {
        {
            MaybeExclusiveLock lock(mLOSCacheMutex, mLockingPolicy);
            mLOSCache.erase(
                std::remove_if(mLOSCache.begin(), mLOSCache.end(), [](const LOSRequest& req) { return req.mStale; }),
                mLOSCache.end());
        }
        mTimeEnd = mTimer->tick();
        if (mWorkersSync != nullptr)
            mWorkersSync->workIsDone();
    }

    void PhysicsTaskScheduler::syncWithMainThread()
    {
        if (mSimulations == nullptr)
            return;
        const Visitors::Sync vis{ mAdvanceSimulation, mTimeAccum, mPhysicsDt, this };
        for (auto& sim : *mSimulations)
            std::visit(vis, sim);
        mSimulations->clear();
        mSimulations = nullptr;
    }

    // Attempt to acquire unique lock on mSimulationMutex while not all worker
    // threads are holding shared lock but will have to may lead to a deadlock because
    // C++ standard does not guarantee priority for exclusive and shared locks
    // for std::shared_mutex. For example microsoft STL implementation points out
    // for the absence of such priority:
    // https://docs.microsoft.com/en-us/windows/win32/sync/slim-reader-writer--srw--locks
    void PhysicsTaskScheduler::waitForWorkers()
    {
        if (mWorkersSync != nullptr)
            mWorkersSync->waitForWorkers();
    }
}
