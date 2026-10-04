#include "physicssystem.hpp"

#include <algorithm>
#include <memory>
#include <vector>

#include <osg/Group>
#include <osg/Stats>
#include <osg/Timer>

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConcaveShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btPolyhedralConvexShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionShapes/btTriangleCallback.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

#include <LinearMath/btQuickprof.h>
#include <LinearMath/btVector3.h>

#include <components/debug/debuglog.hpp>
#include <components/esm3/loadgmst.hpp>
#include <components/esm3/loadmgef.hpp>
#include <components/misc/constants.hpp>
#include <components/misc/convert.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/misc/resourcehelpers.hpp>
#include <components/misc/strings/conversion.hpp>
#include <components/resource/bulletshape.hpp>
#include <components/resource/bulletshapemanager.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/settings/values.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/rotationflags.hpp"
#include "../mwbase/world.hpp"

#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include "../mwmechanics/movement.hpp"

#include "../mwworld/cellstore.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/player.hpp"

#include "../mwrender/bulletdebugdraw.hpp"

#include "../mwworld/class.hpp"

#include "actor.hpp"
#include "collisiontype.hpp"

#include "closestnotmerayresultcallback.hpp"
#include "contacttestresultcallback.hpp"
#include "hasspherecollisioncallback.hpp"
#include "heightfield.hpp"
#include "movementsolver.hpp"
#include "mtphysics.hpp"
#include "object.hpp"
#include "projectile.hpp"
#include "ragdoll.hpp"

namespace
{
    void handleJump(const MWWorld::Ptr& ptr)
    {
        if (!ptr.getClass().isActor())
            return;
        if (ptr.getClass().getMovementSettings(ptr).mPosition[2] == 0)
            return;
        const bool isPlayer = (ptr == MWMechanics::getPlayer());
        // Advance acrobatics and set flag for GetPCJumping
        if (isPlayer)
        {
            ptr.getClass().skillUsageSucceeded(ptr, ESM::Skill::Acrobatics, ESM::Skill::Acrobatics_Jump);
            MWBase::Environment::get().getWorld()->getPlayer().setJumping(true);
        }

        // Decrease fatigue
        if (!isPlayer || !MWBase::Environment::get().getWorld()->getGodModeState())
        {
            const MWWorld::Store<ESM::GameSetting>& gmst
                = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>();
            const float fFatigueJumpBase = gmst.find("fFatigueJumpBase")->mValue.getFloat();
            const float fFatigueJumpMult = gmst.find("fFatigueJumpMult")->mValue.getFloat();
            const float normalizedEncumbrance = std::min(1.f, ptr.getClass().getNormalizedEncumbrance(ptr));
            const float fatigueDecrease = fFatigueJumpBase + normalizedEncumbrance * fFatigueJumpMult;
            MWMechanics::DynamicStat<float> fatigue = ptr.getClass().getCreatureStats(ptr).getFatigue();
            fatigue.setCurrent(fatigue.getCurrent() - fatigueDecrease);
            ptr.getClass().getCreatureStats(ptr).setFatigue(fatigue);
        }
        ptr.getClass().getMovementSettings(ptr).mPosition[2] = 0;
    }

    // Only pairs involving a simulated rigid body are of interest to the dynamics step. Everything else
    // (actors vs. world, statics vs. statics) is handled by explicit queries, so keep it out of the pair cache.
    struct DynamicPairFilter final : public btOverlapFilterCallback
    {
        bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override
        {
            const int groups = proxy0->m_collisionFilterGroup | proxy1->m_collisionFilterGroup;
            if ((groups & MWPhysics::CollisionType_Dynamic) == 0)
                return false;
            // Projectiles find what they hit with their own sweep test; they're not solid to the simulation.
            if ((groups & MWPhysics::CollisionType_Projectile) != 0)
                return false;
            return (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0
                && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask) != 0;
        }
    };

    class DynamicsWorld final : public btDiscreteDynamicsWorld
    {
    public:
        using btDiscreteDynamicsWorld::btDiscreteDynamicsWorld;

        // The default updates the AABB of every active collision object each step. Static objects keep
        // their AABBs updated manually, so only refresh the moving rigid bodies.
        void updateAabbs() override
        {
            for (int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
            {
                btRigidBody* body = m_nonStaticRigidBodies[i];
                if (body->isActive())
                    updateSingleAabb(body);
            }
        }
    };

    // Finds the deepest overlap of a probe object with static geometry, as the move that would resolve it.
    struct DepenetrationCallback final : public btCollisionWorld::ContactResultCallback
    {
        const btCollisionObject* mProbe;
        btVector3 mPush{ 0, 0, 0 };
        btScalar mDeepest = 0;

        explicit DepenetrationCallback(const btCollisionObject* probe)
            : mProbe(probe)
        {
            m_collisionFilterGroup = MWPhysics::CollisionType_Dynamic;
            m_collisionFilterMask
                = MWPhysics::CollisionType_DynamicSupport;
        }

        btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* col0Wrap, int /*partId0*/,
            int /*index0*/, const btCollisionObjectWrapper* /*col1Wrap*/, int /*partId1*/, int /*index1*/) override
        {
            // m_normalWorldOnB points from B towards A.
            const btScalar depth = -cp.getDistance();
            if (depth <= 0.05f || depth <= mDeepest)
                return 0;
            mDeepest = depth;
            const btVector3 towardsProbe
                = col0Wrap->getCollisionObject() == mProbe ? cp.m_normalWorldOnB : -cp.m_normalWorldOnB;
            mPush = towardsProbe * (depth + 0.1f);
            return 0;
        }
    };

    struct VertexCollector final : public btTriangleCallback
    {
        const btTransform& mTransform;
        std::vector<btVector3>& mOut;

        VertexCollector(const btTransform& transform, std::vector<btVector3>& out)
            : mTransform(transform)
            , mOut(out)
        {
        }

        void processTriangle(btVector3* triangle, int /*partId*/, int /*triangleIndex*/) override
        {
            for (int i = 0; i < 3; ++i)
                mOut.push_back(mTransform(triangle[i]));
        }
    };

    // Collect the vertices of a (possibly compound, possibly triangle mesh) shape, transformed by transform.
    void collectVertices(const btCollisionShape& shape, const btTransform& transform, std::vector<btVector3>& out)
    {
        if (shape.isCompound())
        {
            const auto& compound = static_cast<const btCompoundShape&>(shape);
            for (int i = 0; i < compound.getNumChildShapes(); ++i)
                collectVertices(
                    *compound.getChildShape(i), transform * compound.getChildTransform(i), out);
        }
        else if (shape.isConcave())
        {
            VertexCollector collector(transform, out);
            const btVector3 everywhere(1e6, 1e6, 1e6);
            static_cast<const btConcaveShape&>(shape).processAllTriangles(&collector, -everywhere, everywhere);
        }
        else if (shape.isPolyhedral())
        {
            const auto& polyhedron = static_cast<const btPolyhedralConvexShape&>(shape);
            for (int i = 0; i < polyhedron.getNumVertices(); ++i)
            {
                btVector3 vertex;
                polyhedron.getVertex(i, vertex);
                out.push_back(transform(vertex));
            }
        }
        else
        {
            btVector3 aabbMin;
            btVector3 aabbMax;
            shape.getAabb(transform, aabbMin, aabbMax);
            for (int i = 0; i < 8; ++i)
                out.emplace_back(
                    (i & 1) ? aabbMax.x() : aabbMin.x(), (i & 2) ? aabbMax.y() : aabbMin.y(),
                    (i & 4) ? aabbMax.z() : aabbMin.z());
        }
    }

    // Inverse of Misc::Convert::makeOsgQuat: returns the ESM rotation (x, y, z) that produces the given quaternion.
    // makeOsgQuat(rot) is the rotation Rx(-x) * Ry(-y) * Rz(-z) (applied right to left).
    osg::Vec3f toEsmRotation(const osg::Quat& quat)
    {
        const btMatrix3x3 m(Misc::Convert::toBullet(quat));
        const btScalar sinB = std::clamp(m[0][2], btScalar(-1), btScalar(1));
        const btScalar b = std::asin(sinB);
        btScalar a;
        btScalar c;
        if (std::abs(sinB) < 0.9999999)
        {
            a = std::atan2(-m[1][2], m[2][2]);
            c = std::atan2(-m[0][1], m[0][0]);
        }
        else
        {
            // Gimbal lock: only a + c (or a - c) is determined, so put it all in a.
            a = std::atan2(m[2][1], m[1][1]);
            c = 0;
        }
        return osg::Vec3f(static_cast<float>(-a), static_cast<float>(-b), static_cast<float>(-c));
    }
}

namespace MWPhysics
{
    osg::Vec3f quatToEsmRotation(const osg::Quat& quat)
    {
        return toEsmRotation(quat);
    }

    PhysicsSystem::PhysicsSystem(Resource::ResourceSystem* resourceSystem, osg::ref_ptr<osg::Group> parentNode)
        : mPhysicsDt(1.f / 60.f)
        , mShapeManager(std::make_unique<Resource::BulletShapeManager>(resourceSystem->getVFS(),
              resourceSystem->getSceneManager(), resourceSystem->getNifFileManager(),
              Settings::cells().mCacheExpiryDelay))
        , mResourceSystem(resourceSystem)
        , mDebugDrawEnabled(false)
        , mTimeAccum(0.0f)
        , mProjectileId(0)
        , mWaterHeight(0)
        , mWaterEnabled(false)
        , mParentNode(std::move(parentNode))
    {
        mResourceSystem->addResourceManager(mShapeManager.get());

        mCollisionConfiguration = std::make_unique<btDefaultCollisionConfiguration>();
        mDispatcher = std::make_unique<btCollisionDispatcher>(mCollisionConfiguration.get());
        mBroadphase = std::make_unique<btDbvtBroadphase>();
        mOverlapFilter = std::make_unique<DynamicPairFilter>();
        mBroadphase->getOverlappingPairCache()->setOverlapFilterCallback(mOverlapFilter.get());
        mConstraintSolver = std::make_unique<btSequentialImpulseConstraintSolver>();

        mCollisionWorld = std::make_unique<DynamicsWorld>(
            mDispatcher.get(), mBroadphase.get(), mConstraintSolver.get(), mCollisionConfiguration.get());
        mCollisionWorld->setGravity(btVector3(0, 0, -Constants::GravityConst * Constants::UnitsPerMeter));
        // Ragdolls are long chains of joints; more solver passes keep them from stretching.
        mCollisionWorld->getSolverInfo().m_numIterations = 20;

        // Don't update AABBs of all objects every frame. Most objects in MW are static, so we don't need this.
        // Should a "static" object ever be moved, we have to update its AABB manually using
        // DynamicsWorld::updateSingleAabb.
        mCollisionWorld->setForceUpdateAllAabbs(false);

        // Check if a user decided to override a physics system FPS
        if (const char* env = getenv("OPENMW_PHYSICS_FPS"))
        {
            if (const auto physFramerate = Misc::StringUtils::toNumeric<float>(env);
                physFramerate.has_value() && *physFramerate > 0)
            {
                mPhysicsDt = 1.f / *physFramerate;
                Log(Debug::Warning) << "Warning: using custom physics framerate (" << *physFramerate << " FPS).";
            }
        }

        mDebugDrawer = std::make_unique<MWRender::DebugDrawer>(mParentNode, mCollisionWorld.get(), mDebugDrawEnabled);
        mTaskScheduler = std::make_unique<PhysicsTaskScheduler>(mPhysicsDt, mCollisionWorld.get(), mDebugDrawer.get());
    }

    PhysicsSystem::~PhysicsSystem()
    {
        mResourceSystem->removeResourceManager(mShapeManager.get());

        if (mWaterCollisionObject)
            mTaskScheduler->removeCollisionObject(mWaterCollisionObject.get());

        mTaskScheduler->releaseSharedStates();
        mHeightFields.clear();
        mRagdolls.clear();
        mDynamicObjects.clear();
        mObjects.clear();
        mActors.clear();
        mProjectiles.clear();
    }

    Resource::BulletShapeManager* PhysicsSystem::getShapeManager()
    {
        return mShapeManager.get();
    }

    bool PhysicsSystem::toggleDebugRendering()
    {
        mDebugDrawEnabled = !mDebugDrawEnabled;

        mCollisionWorld->setDebugDrawer(mDebugDrawEnabled ? mDebugDrawer.get() : nullptr);
        mDebugDrawer->setDebugMode(mDebugDrawEnabled);
        return mDebugDrawEnabled;
    }

    void PhysicsSystem::markAsNonSolid(const MWWorld::ConstPtr& ptr)
    {
        ObjectMap::iterator found = mObjects.find(ptr.mRef);
        if (found == mObjects.end())
            return;

        found->second->setSolid(false);
    }

    bool PhysicsSystem::isOnSolidGround(const MWWorld::Ptr& actor) const
    {
        const Actor* physactor = getActor(actor);
        if (!physactor || !physactor->getOnGround() || !physactor->getCollisionMode())
            return false;

        const auto obj = physactor->getStandingOnPtr();
        if (obj.isEmpty())
            return true; // assume standing on terrain (which is a non-object, so not collision tracked)

        ObjectMap::const_iterator foundObj = mObjects.find(obj.mRef);
        if (foundObj == mObjects.end())
            return false;

        if (!foundObj->second->isSolid())
            return false;

        return true;
    }

    RayCastingResult PhysicsSystem::castRay(const osg::Vec3f& from, const osg::Vec3f& to,
        const std::vector<MWWorld::ConstPtr>& ignore, const std::vector<MWWorld::Ptr>& targets, int mask,
        int group) const
    {
        if (from == to)
        {
            RayCastingResult result;
            result.mHit = false;
            return result;
        }
        btVector3 btFrom = Misc::Convert::toBullet(from);
        btVector3 btTo = Misc::Convert::toBullet(to);

        std::vector<const btCollisionObject*> ignoreList;
        std::vector<const btCollisionObject*> targetCollisionObjects;

        for (const auto& ptr : ignore)
        {
            if (!ptr.isEmpty())
            {
                const Actor* actor = getActor(ptr);
                if (actor)
                    ignoreList.push_back(actor->getCollisionObject());
                else
                {
                    const Object* object = getObject(ptr);
                    if (object)
                        ignoreList.push_back(object->getCollisionObject());
                }
            }
        }

        if (!targets.empty())
        {
            for (const MWWorld::Ptr& target : targets)
            {
                const Actor* actor = getActor(target);
                if (actor)
                    targetCollisionObjects.push_back(actor->getCollisionObject());
            }
        }

        ClosestNotMeRayResultCallback resultCallback(ignoreList, targetCollisionObjects, btFrom, btTo);
        resultCallback.m_collisionFilterGroup = group;
        resultCallback.m_collisionFilterMask = mask;

        mTaskScheduler->rayTest(btFrom, btTo, resultCallback);

        RayCastingResult result;
        result.mHit = resultCallback.hasHit();
        if (resultCallback.hasHit())
        {
            result.mHitPos = Misc::Convert::toOsg(resultCallback.m_hitPointWorld);
            result.mHitNormal = Misc::Convert::toOsg(resultCallback.m_hitNormalWorld);
            if (PtrHolder* ptrHolder = static_cast<PtrHolder*>(resultCallback.m_collisionObject->getUserPointer()))
                result.mHitObject = ptrHolder->getPtr();
        }
        return result;
    }

    RayCastingResult PhysicsSystem::castSphere(
        const osg::Vec3f& from, const osg::Vec3f& to, float radius, int mask, int group) const
    {
        btCollisionWorld::ClosestConvexResultCallback callback(
            Misc::Convert::toBullet(from), Misc::Convert::toBullet(to));
        callback.m_collisionFilterGroup = group;
        callback.m_collisionFilterMask = mask;

        btSphereShape shape(radius);
        const btQuaternion btrot = btQuaternion::getIdentity();

        mTaskScheduler->convexSweepTest(&shape, btTransform(btrot, Misc::Convert::toBullet(from)),
            btTransform(btrot, Misc::Convert::toBullet(to)), callback);

        RayCastingResult result;
        result.mHit = callback.hasHit();
        if (result.mHit)
        {
            result.mHitPos = Misc::Convert::toOsg(callback.m_hitPointWorld);
            result.mHitNormal = Misc::Convert::toOsg(callback.m_hitNormalWorld);
            if (auto* ptrHolder = static_cast<PtrHolder*>(callback.m_hitCollisionObject->getUserPointer()))
                result.mHitObject = ptrHolder->getPtr();
        }
        return result;
    }

    bool PhysicsSystem::getLineOfSight(const MWWorld::ConstPtr& actor1, const MWWorld::ConstPtr& actor2) const
    {
        if (actor1 == actor2)
            return true;

        const auto it1 = mActors.find(actor1.mRef);
        const auto it2 = mActors.find(actor2.mRef);
        if (it1 == mActors.end() || it2 == mActors.end())
            return false;

        return mTaskScheduler->getLineOfSight(it1->second, it2->second);
    }

    bool PhysicsSystem::isOnGround(const MWWorld::Ptr& actor)
    {
        Actor* physactor = getActor(actor);
        return physactor && physactor->getOnGround() && physactor->getCollisionMode();
    }

    bool PhysicsSystem::canMoveToWaterSurface(const MWWorld::ConstPtr& actor, const float waterlevel)
    {
        const auto* physactor = getActor(actor);
        return physactor && physactor->canMoveToWaterSurface(waterlevel, mCollisionWorld.get());
    }

    osg::Vec3f PhysicsSystem::getHalfExtents(const MWWorld::ConstPtr& actor) const
    {
        const Actor* physactor = getActor(actor);
        if (physactor)
            return physactor->getHalfExtents();
        else
            return osg::Vec3f();
    }

    osg::Vec3f PhysicsSystem::getOriginalHalfExtents(const MWWorld::ConstPtr& actor) const
    {
        if (const Actor* physactor = getActor(actor))
            return physactor->getOriginalHalfExtents();
        else
            return osg::Vec3f();
    }

    osg::Vec3f PhysicsSystem::getRenderingHalfExtents(const MWWorld::ConstPtr& actor) const
    {
        const Actor* physactor = getActor(actor);
        if (physactor)
            return physactor->getRenderingHalfExtents();
        else
            return osg::Vec3f();
    }

    osg::BoundingBox PhysicsSystem::getBoundingBox(const MWWorld::ConstPtr& object) const
    {
        const Object* physobject = getObject(object);
        if (!physobject)
            return osg::BoundingBox();
        btVector3 min, max;
        mTaskScheduler->getAabb(physobject->getCollisionObject(), min, max);
        return osg::BoundingBox(Misc::Convert::toOsg(min), Misc::Convert::toOsg(max));
    }

    osg::Vec3f PhysicsSystem::getCollisionObjectPosition(const MWWorld::ConstPtr& actor) const
    {
        const Actor* physactor = getActor(actor);
        if (physactor)
            return physactor->getCollisionObjectPosition();
        else
            return osg::Vec3f();
    }

    std::vector<ContactPoint> PhysicsSystem::getCollisionsPoints(
        const MWWorld::ConstPtr& ptr, int collisionGroup, int collisionMask) const
    {
        btCollisionObject* me = nullptr;

        auto found = mObjects.find(ptr.mRef);
        if (found != mObjects.end())
            me = found->second->getCollisionObject();
        else
            return {};

        ContactTestResultCallback resultCallback(me);
        resultCallback.m_collisionFilterGroup = collisionGroup;
        resultCallback.m_collisionFilterMask = collisionMask;
        mTaskScheduler->contactTest(me, resultCallback);
        return resultCallback.mResult;
    }

    std::vector<MWWorld::Ptr> PhysicsSystem::getCollisions(
        const MWWorld::ConstPtr& ptr, int collisionGroup, int collisionMask) const
    {
        std::vector<MWWorld::Ptr> actors;
        for (auto& [actor, point, normal] : getCollisionsPoints(ptr, collisionGroup, collisionMask))
            actors.emplace_back(actor);
        return actors;
    }

    osg::Vec3f PhysicsSystem::traceDown(const MWWorld::Ptr& ptr, const osg::Vec3f& position, float maxHeight)
    {
        ActorMap::iterator found = mActors.find(ptr.mRef);
        if (found == mActors.end())
            return ptr.getRefData().getPosition().asVec3();
        return MovementSolver::traceDown(ptr, position, found->second.get(), mCollisionWorld.get(), maxHeight);
    }

    void PhysicsSystem::addHeightField(const float* heights, int x, int y, int size, int verts, float minH, float maxH,
        std::shared_ptr<const ESMTerrain::LandObject> holdObject)
    {
        mHeightFields[std::make_pair(x, y)] = std::make_unique<HeightField>(
            heights, x, y, size, verts, minH, maxH, std::move(holdObject), mTaskScheduler.get());
    }

    void PhysicsSystem::removeHeightField(int x, int y)
    {
        HeightFieldMap::iterator heightfield = mHeightFields.find(std::make_pair(x, y));
        if (heightfield != mHeightFields.end())
            mHeightFields.erase(heightfield);
    }

    const HeightField* PhysicsSystem::getHeightField(int x, int y) const
    {
        const auto heightField = mHeightFields.find(std::make_pair(x, y));
        if (heightField == mHeightFields.end())
            return nullptr;
        return heightField->second.get();
    }

    void PhysicsSystem::addObject(
        const MWWorld::Ptr& ptr, VFS::Path::NormalizedView mesh, osg::Quat rotation, int collisionType)
    {
        if (ptr.mRef->mData.mPhysicsPostponed)
            return;

        const VFS::Path::Normalized animationMesh = ptr.getClass().useAnim()
            ? Misc::ResourceHelpers::correctActorModelPath(mesh, mResourceSystem->getVFS())
            : VFS::Path::Normalized(mesh);
        std::shared_ptr<Resource::BulletShapeInstance> shapeInstance = mShapeManager->getInstance(animationMesh);
        if (!shapeInstance || !shapeInstance->mCollisionShape)
            return;

        assert(!getObject(ptr));

        // Override collision type based on shape content.
        switch (shapeInstance->mVisualCollisionType)
        {
            case Resource::VisualCollisionType::None:
                break;
            case Resource::VisualCollisionType::Default:
                collisionType = CollisionType_VisualOnly;
                break;
            case Resource::VisualCollisionType::Camera:
                collisionType = CollisionType_CameraOnly;
                break;
        }

        auto obj = std::make_shared<Object>(ptr, shapeInstance, rotation, collisionType, mTaskScheduler.get());
        mObjects.emplace(ptr.mRef, obj);

        if (obj->isAnimated())
            mAnimatedObjects.emplace(obj.get(), false);
        else if (collisionType == CollisionType_World && shapeInstance->getSource()->mHasCollisionNode)
        {
            // Furniture often has a simplified collision mesh (a bookcase may be a hollow box). Simulated
            // objects need the real shape to rest on its shelves, so give them the rendered geometry instead.
            // Large objects (buildings, rocks) keep the regular collision: too costly, and rarely needed.
            if (std::shared_ptr<const Resource::BulletShape> visible = mShapeManager->getVisibleShape(animationMesh))
            {
                btVector3 aabbMin;
                btVector3 aabbMax;
                visible->mCollisionShape->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
                const btVector3 size = (aabbMax - aabbMin) * ptr.getCellRef().getScale();
                constexpr btScalar maxFurnitureSize = 400.f;
                if (size[size.maxAxis()] <= maxFurnitureSize)
                    obj->setDetailShape(Resource::makeInstance(std::move(visible)));
            }
        }
    }

    const std::vector<btVector3>& PhysicsSystem::getHullPoints(
        VFS::Path::NormalizedView mesh, const Resource::BulletShape& shape)
    {
        const std::string key(mesh.value());
        if (const auto found = mHullCache.find(key); found != mHullCache.end())
            return found->second;

        std::vector<btVector3> vertices;
        collectVertices(*shape.mCollisionShape, btTransform::getIdentity(), vertices);
        std::vector<btVector3>& hull = mHullCache[key];
        if (vertices.size() >= 4)
        {
            // Reduce the (possibly thousands of) mesh vertices to a small hull that is cheap to simulate.
            const btConvexHullShape allPoints(
                vertices.front().m_floats, static_cast<int>(vertices.size()), sizeof(btVector3));
            btShapeHull shapeHull(&allPoints);
            if (shapeHull.buildHull(allPoints.getMargin()))
                hull.assign(shapeHull.getVertexPointer(), shapeHull.getVertexPointer() + shapeHull.numVertices());
        }
        return hull;
    }

    void PhysicsSystem::addDynamicObject(
        const MWWorld::Ptr& ptr, VFS::Path::NormalizedView mesh, osg::Quat rotation, float mass, bool metal,
        bool projectile)
    {
        if (ptr.mRef->mData.mPhysicsPostponed)
            return;

        std::shared_ptr<Resource::BulletShapeInstance> shapeInstance = mShapeManager->getInstance(mesh);
        if (!shapeInstance || !shapeInstance->mCollisionShape)
        {
            if (mWarnedNoShape.emplace(mesh.value()).second)
                Log(Debug::Warning) << "No collision shape for dynamic object " << ptr.toString() << " ("
                                    << mesh.value() << ")";
            return;
        }

        assert(!getObject(ptr));

        // Fit the hull to what the player sees; the collision mesh can be smaller (items poking into floors).
        const std::shared_ptr<const Resource::BulletShape> visible = mShapeManager->getVisibleShape(mesh);
        const std::vector<btVector3>& hullPoints
            = getHullPoints(mesh, visible != nullptr ? *visible : *shapeInstance->getSource());
        auto obj = std::make_shared<Object>(
            ptr, shapeInstance, rotation, mass, metal, hullPoints, mTaskScheduler.get());

        // Whether ammunition was stuck in something isn't saved, but it shows: it is embedded in the surface.
        if (projectile)
        {
            DepenetrationCallback embedded(obj->getCollisionObject());
            mTaskScheduler->contactTest(obj->getCollisionObject(), embedded);
            constexpr btScalar minStuckDepth = 1.5f;
            if (embedded.mDeepest > minStuckDepth)
            {
                obj->requestStuck(true);
                mTaskScheduler->updateSingleAabb(obj);
            }
        }

        mObjects.emplace(ptr.mRef, obj);
        mDynamicObjects.push_back(std::move(obj));
    }

    bool PhysicsSystem::createRagdoll(
        const MWWorld::Ptr& actor, const std::map<std::string, osg::Matrixf, std::less<>>& bones, const osg::Vec3f& kick)
    {
        for (const std::string& bone : Ragdoll::getRequiredBones())
            if (bones.find(bone) == bones.end())
                return false;
        // Falling as it was moving.
        const osg::Vec3f velocity = getDeathVelocity(actor);
        mRagdolls.erase(actor.mRef);
        mRagdolls.emplace(actor.mRef, std::make_unique<Ragdoll>(actor, bones, velocity, kick, mTaskScheduler.get()));
        return true;
    }

    bool PhysicsSystem::createSkinnedRagdoll(
        const MWWorld::Ptr& actor, const std::vector<Ragdoll::SkinnedBone>& bones, const osg::Vec3f& kick)
    {
        const osg::Vec3f velocity = getDeathVelocity(actor);
        std::unique_ptr<Ragdoll> ragdoll = Ragdoll::fromSkeleton(actor, bones, velocity, kick, mTaskScheduler.get());
        if (ragdoll == nullptr)
            return false;
        mRagdolls.erase(actor.mRef);
        mRagdolls.emplace(actor.mRef, std::move(ragdoll));
        return true;
    }

    void PhysicsSystem::settleRagdoll(const MWWorld::ConstPtr& actor)
    {
        if (const auto found = mRagdolls.find(actor.mRef); found != mRagdolls.end())
            found->second->setAtRest();
    }

    std::vector<std::pair<std::string, osg::Matrixf>> PhysicsSystem::getLastRagdollBonePoses(
        const MWWorld::ConstPtr& actor) const
    {
        const auto found = mRagdolls.find(actor.mRef);
        if (found == mRagdolls.end())
            return {};
        return found->second->getLastBonePoses();
    }

    bool PhysicsSystem::createRigidPieceRagdoll(
        const MWWorld::Ptr& actor, const std::vector<Ragdoll::RigidPiece>& pieces, const osg::Vec3f& kick)
    {
        std::unique_ptr<Ragdoll> ragdoll
            = Ragdoll::fromRigidPieces(actor, pieces, getDeathVelocity(actor), kick, mTaskScheduler.get());
        if (ragdoll == nullptr)
            return false;
        mRagdolls.erase(actor.mRef);
        mRagdolls.emplace(actor.mRef, std::move(ragdoll));
        return true;
    }

    osg::Vec3f PhysicsSystem::getDeathVelocity(const MWWorld::ConstPtr& actor) const
    {
        // Falling as it was moving.
        const auto found = mActors.find(actor.mRef);
        if (found == mActors.end())
            return osg::Vec3f();
        osg::Vec3f velocity = (found->second->getPosition() - found->second->getPreviousPosition()) / mPhysicsDt;
        constexpr float maxSpeed = 600.f;
        if (velocity.length2() > maxSpeed * maxSpeed)
            velocity *= maxSpeed / velocity.length();
        return velocity;
    }

    bool PhysicsSystem::createCorpseBody(const MWWorld::Ptr& actor)
    {
        const auto found = mActors.find(actor.mRef);
        const SceneUtil::PositionAttitudeTransform* base = actor.getRefData().getBaseNode();
        if (found == mActors.end() || base == nullptr)
            return false;
        // The standing size, but lower (dead things lie down) and a little narrower (the box is roomier than the
        // body, and its corners would hold it up off slopes).
        osg::Vec3f half = found->second->getHalfExtents();
        half *= 0.8f;
        half.z() *= 0.75f;
        half.x() = std::max(half.x(), 4.f);
        half.y() = std::max(half.y(), 4.f);
        half.z() = std::max(half.z(), 4.f);
        const btTransform bodyWorld(Misc::Convert::toBullet(base->getAttitude()),
            Misc::Convert::toBullet(base->getPosition() + osg::Vec3f(0, 0, half.z())));
        // About half as dense as water (~70 units to a meter), allowing for the box being roomier than the body.
        const float volume = 8.f * half.x() * half.y() * half.z() / (70.f * 70.f * 70.f);
        const float mass = std::clamp(volume * 500.f, 3.f, 400.f);
        const osg::Matrixf baseWorld = osg::Matrixf::scale(base->getScale())
            * osg::Matrixf::rotate(base->getAttitude()) * osg::Matrixf::translate(base->getPosition());
        mRagdolls.erase(actor.mRef);
        mRagdolls.emplace(
            actor.mRef, std::make_unique<Ragdoll>(actor, bodyWorld, half, mass, baseWorld, mTaskScheduler.get()));
        return true;
    }

    std::vector<std::pair<std::string, osg::Matrixf>> PhysicsSystem::getRagdollBonePoses(
        const MWWorld::ConstPtr& actor) const
    {
        const auto found = mRagdolls.find(actor.mRef);
        if (found == mRagdolls.end())
            return {};
        return found->second->getBonePoses(std::clamp(mTimeAccum / mPhysicsDt, 0.f, 1.f));
    }

    void PhysicsSystem::removeRagdoll(const MWWorld::ConstPtr& actor)
    {
        mRagdolls.erase(actor.mRef);
    }

    bool PhysicsSystem::hasRagdoll(const MWWorld::ConstPtr& actor) const
    {
        return mRagdolls.find(actor.mRef) != mRagdolls.end();
    }

    void PhysicsSystem::wakeNewObject(const MWWorld::Ptr& ptr)
    {
        const auto found = mObjects.find(ptr.mRef);
        if (found == mObjects.end() || !found->second->isDynamic())
            return;
        placeOnSurface(*found->second);
        found->second->requestWake();
        mTaskScheduler->updateSingleAabb(found->second);
    }

    void PhysicsSystem::stickObject(const MWWorld::Ptr& ptr, const osg::Vec3f& hitPoint)
    {
        const auto found = mObjects.find(ptr.mRef);
        if (found == mObjects.end() || !found->second->isDynamic())
            return;
        Object& object = *found->second;

        // Its tip a fifth of its length past the hit point. Exactly there (no placing on surfaces: it's meant to
        // be embedded), and fixed until taken or grabbed.
        const auto [back, tip] = object.getDynamicShapeYRange();
        placeTipAt(object, hitPoint, std::clamp((tip - back) * 0.2f, 4.f, 12.f));
        object.requestStuck(true);
        mTaskScheduler->updateSingleAabb(found->second);
    }

    void PhysicsSystem::deflectObject(const MWWorld::Ptr& ptr, const osg::Vec3f& hitPoint, const osg::Vec3f& velocity)
    {
        const auto found = mObjects.find(ptr.mRef);
        if (found == mObjects.end() || !found->second->isDynamic())
            return;
        // Its tip just short of the hit point, so it doesn't start inside what it glanced off, then away.
        placeTipAt(*found->second, hitPoint, -2.f);
        found->second->requestVelocity(velocity);
        found->second->requestWake();
        mTaskScheduler->updateSingleAabb(found->second);
    }

    void PhysicsSystem::placeTipAt(Object& object, const osg::Vec3f& point, float depth)
    {
        // The tip is the far end along the object's Y axis, which points where it flew; the model's origin may be
        // anywhere along it.
        const float tip = object.getDynamicShapeYRange().second;
        const btQuaternion rotation = object.getTransform().getRotation();
        const osg::Vec3f forward = Misc::Convert::toOsg(btMatrix3x3(rotation) * btVector3(0, 1, 0));
        const osg::Vec3f origin = point + forward * depth - forward * tip;
        MWBase::Environment::get().getWorld()->moveObject(object.getPtr(), origin, false, false);
        object.updatePosition();
    }

    void PhysicsSystem::launchObject(const MWWorld::Ptr& ptr, const osg::Vec3f& velocity)
    {
        const auto found = mObjects.find(ptr.mRef);
        if (found == mObjects.end() || !found->second->isDynamic())
            return;
        found->second->requestVelocity(velocity);
        found->second->requestWake();
        mTaskScheduler->updateSingleAabb(found->second);
    }

    bool PhysicsSystem::canHoldObject(const MWWorld::ConstPtr& ptr) const
    {
        const Object* object = getObject(ptr);
        return (object != nullptr && object->isDynamic()) || hasRagdoll(ptr);
    }

    namespace
    {
        // Rotation about the vertical that turns the default view direction (+Y) into direction.
        btQuaternion viewYaw(const osg::Vec3f& direction)
        {
            return btQuaternion(btVector3(0, 0, 1), std::atan2(-direction.x(), direction.y()));
        }
    }

    bool PhysicsSystem::holdObject(const MWWorld::Ptr& ptr, const osg::Vec3f& eye, const osg::Vec3f& viewDirection)
    {
        if (const auto ragdoll = mRagdolls.find(ptr.mRef); ragdoll != mRagdolls.end())
        {
            const std::shared_ptr<PtrHolder> part = ragdoll->second->findPart(eye, viewDirection);
            if (part == nullptr)
                return false;
            mHoldDistance = 70.f;
            mHoldingRagdoll = true;
            // A one-piece body hangs from where it was grabbed (and drags along the ground).
            if (const std::optional<osg::Vec3f> grabPoint = ragdoll->second->getGrabPoint(eye))
            {
                mTaskScheduler->holdObject(part, false, 0, Misc::Convert::toBullet(*grabPoint));
                return true;
            }
            // Gripped firmly enough to drag the rest of the body along (a heavy beast takes a heavier grip); it
            // hangs as it likes. A loose piece is just carried.
            const btScalar gripMass
                = ragdoll->second->isJointed() ? std::clamp(ragdoll->second->getMass() * 0.6f, 30.f, 300.f) : 0.f;
            mTaskScheduler->holdObject(part, false, gripMass);
            return true;
        }

        const auto found = mObjects.find(ptr.mRef);
        if (found == mObjects.end() || !found->second->isDynamic())
            return false;
        const std::shared_ptr<Object>& object = found->second;

        // Hold it far enough out that it doesn't fill the view.
        btVector3 aabbMin;
        btVector3 aabbMax;
        object->getRigidBody()->getCollisionShape()->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
        const float radius = static_cast<float>((aabbMax - aabbMin).length() * 0.5);
        mHoldDistance = std::max(70.f, 40.f + radius);

        // Keep the orientation it has relative to the viewer, so it turns along when the viewer turns.
        mHoldRelativeRotation = viewYaw(viewDirection).inverse() * object->getTransform().getRotation();

        mHoldingRagdoll = false;
        mTaskScheduler->holdObject(object, true, 0);
        return true;
    }

    void PhysicsSystem::setHoldView(const osg::Vec3f& eye, const osg::Vec3f& direction)
    {
        osg::Vec3f target = eye + direction * mHoldDistance;
        // A body is dragged at about waist height, not lifted up to the face.
        if (mHoldingRagdoll)
            target.z() = std::min(target.z(), eye.z() - 60.f);
        mTaskScheduler->setHoldTarget(Misc::Convert::toBullet(target), viewYaw(direction) * mHoldRelativeRotation);
    }

    void PhysicsSystem::rotateHeldObject(float yaw, float pitch)
    {
        // In the viewer's frame (view yaw removed): spin around the vertical, tilt around the sideways axis.
        mHoldRelativeRotation = btQuaternion(btVector3(0, 0, 1), yaw) * btQuaternion(btVector3(1, 0, 0), pitch)
            * mHoldRelativeRotation;
        mHoldRelativeRotation.normalize();
    }

    void PhysicsSystem::releaseHeldObject(bool throwObject, const osg::Vec3f& direction)
    {
        if (!throwObject)
        {
            mTaskScheduler->releaseHeldObject(std::nullopt);
            return;
        }
        // Bodies are only dragged, not thrown.
        const std::shared_ptr<Object> held = std::dynamic_pointer_cast<Object>(mTaskScheduler->getHeldObject());
        if (held == nullptr)
        {
            mTaskScheduler->releaseHeldObject(std::nullopt);
            return;
        }
        // Light things fly fast, heavy things barely leave the hand.
        const float mass = static_cast<float>(1.0 / held->getRigidBody()->getInvMass());
        const float speed = std::clamp(1200.f * std::sqrt(2.f / mass), 250.f, 1200.f);
        mTaskScheduler->releaseHeldObject(Misc::Convert::toBullet(direction * speed));

        // Only the player carries things, so the player threw it.
        const Actor* thrower = getActor(MWMechanics::getPlayer());
        mTaskScheduler->addFlyingObject(held, thrower != nullptr ? thrower->getCollisionObject() : nullptr);
    }

    std::vector<ThrownObjectHit> PhysicsSystem::takeThrownObjectHits()
    {
        std::vector<ThrownObjectHit> result;
        for (const PhysicsTaskScheduler::FlyingObjectHit& hit : mTaskScheduler->takeFlyingObjectHits())
        {
            const std::shared_ptr<Object> object = hit.mObject.lock();
            if (object == nullptr)
                continue;
            const auto victim = std::find_if(mActors.begin(), mActors.end(),
                [&](const auto& actor) { return actor.second->getCollisionObject() == hit.mActor; });
            if (victim == mActors.end())
                continue;
            result.push_back({ victim->second->getPtr(), object->getPtr(), static_cast<float>(hit.mSpeed) });
        }
        return result;
    }

    bool PhysicsSystem::isHoldingObject() const
    {
        return mTaskScheduler->getHeldObject() != nullptr;
    }

    bool PhysicsSystem::strikeObject(const MWWorld::Ptr& ptr, const osg::Vec3f& velocityChange,
        const osg::Vec3f& point, const osg::Vec3f& source)
    {
        const auto found = mObjects.find(ptr.mRef);
        if (found == mObjects.end() || !found->second->isDynamic())
            return false;
        mTaskScheduler->strikeObjects({ { found->second, Misc::Convert::toBullet(velocityChange),
            Misc::Convert::toBullet(point), Misc::Convert::toBullet(source) } });
        return true;
    }

    std::vector<PhysicsSystem::DynamicObjectInfo> PhysicsSystem::getDynamicObjectsInRange(
        const osg::Vec3f& center, float radius) const
    {
        std::vector<DynamicObjectInfo> result;
        for (const std::shared_ptr<Object>& object : mDynamicObjects)
        {
            const osg::Vec3f position = Misc::Convert::toOsg(object->getCenterOfMassTransform().getOrigin());
            if ((position - center).length2() > radius * radius)
                continue;
            btVector3 aabbMin;
            btVector3 aabbMax;
            object->getDynamicShape()->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
            result.push_back({ object->getPtr(), position, static_cast<float>((aabbMax - aabbMin).length() * 0.5) });
        }
        std::sort(result.begin(), result.end(), [&](const DynamicObjectInfo& a, const DynamicObjectInfo& b) {
            return (a.mCenter - center).length2() < (b.mCenter - center).length2();
        });
        return result;
    }

    void PhysicsSystem::explode(const osg::Vec3f& center, float radius, float speed)
    {
        if (radius <= 0)
            return;
        std::vector<PhysicsTaskScheduler::Strike> strikes;
        for (const std::shared_ptr<Object>& object : mDynamicObjects)
        {
            const osg::Vec3f position = Misc::Convert::toOsg(object->getCenterOfMassTransform().getOrigin());
            osg::Vec3f away = position - center;
            const float distance = away.normalize();
            if (distance >= radius)
                continue;
            // Mostly outwards, a bit upwards, so things on the floor get tossed rather than slid.
            away.z() += 0.5f;
            away.normalize();
            const float falloff = 1.f - distance / radius;
            strikes.push_back({ object, Misc::Convert::toBullet(away * (speed * falloff)),
                object->getCenterOfMassTransform().getOrigin(), Misc::Convert::toBullet(center) });
        }
        if (!strikes.empty())
            mTaskScheduler->strikeObjects(strikes);
    }

    void PhysicsSystem::placeOnSurface(Object& object)
    {
        // Objects placed or moved by the game (PlaceAtPC, snapping to ground) are positioned by their origin,
        // which for most meshes is the center, or may even be inside other geometry (e.g. a stair step). The
        // simulation would push such objects out through the wrong side. Lift the shape onto the surface instead.
        const auto [bottom, top] = object.getDynamicShapeHeightRange();
        const osg::Vec3f position = object.getPtr().getRefData().getPosition().asVec3();
        float lift = 0;

        // Terrain can't be overhead, so search it from high above (handles objects buried in a hillside).
        const RayCastingResult terrain = castRay(osg::Vec3f(position.x(), position.y(), bottom + 2000.f),
            osg::Vec3f(position.x(), position.y(), bottom), {}, {}, CollisionType_HeightMap);
        if (terrain.mHit)
            lift = std::max(lift, terrain.mHitPos.z() - bottom);

        // Other surfaces: is the object's bottom inside something solid (half below a floor, deep inside a
        // staircase)? Then looking up and looking down from above both first meet the same surface: its top.
        // Under a table they meet different ones (the underside and the top), so it stays put.
        constexpr float searchHeight = 200.f;
        constexpr int supportMask = CollisionType_World | CollisionType_Door | CollisionType_DynamicDetail;
        const float from = bottom + lift;
        const RayCastingResult up = castRay(osg::Vec3f(position.x(), position.y(), from),
            osg::Vec3f(position.x(), position.y(), from + searchHeight), {}, {}, supportMask, CollisionType_Dynamic);
        bool inside = false;
        RayCastingResult down;
        if (up.mHit)
        {
            // Start a little above that surface, so a floor further up (stairs to a gallery) isn't in the way.
            const float downFrom = up.mHitPos.z() + 30.f;
            down = castRay(osg::Vec3f(position.x(), position.y(), downFrom),
                osg::Vec3f(position.x(), position.y(), from), {}, {}, supportMask, CollisionType_Dynamic);
            inside = down.mHit && std::abs(up.mHitPos.z() - down.mHitPos.z()) < 1.f;
        }
        if (inside)
            lift = std::max(lift, down.mHitPos.z() - bottom);

        btVector3 offset(0, 0, lift > 0 ? lift + 1.f : 0.f);

        // The rays above only look straight up and down through the middle. The shape can still overlap
        // things off-center, like the front of the next stair step. Push it out along the contacts.
        btCollisionObject probe;
        probe.setCollisionShape(object.getDynamicShape());
        const btTransform centerOfMass = object.getCenterOfMassTransform();
        for (int i = 0; i < 4; ++i)
        {
            probe.setWorldTransform(btTransform(centerOfMass.getRotation(), centerOfMass.getOrigin() + offset));
            DepenetrationCallback depenetration(&probe);
            mTaskScheduler->contactTest(&probe, depenetration);
            if (depenetration.mPush.isZero())
                break;
            offset += depenetration.mPush;
        }

        if (!offset.isZero())
        {
            Log(Debug::Verbose) << "[physics] placed " << object.getPtr().getCellRef().getRefId() << ": lifted "
                             << lift << ", pushed out by (" << offset.x() << ", " << offset.y() << ", "
                             << offset.z() << ")" << (terrain.mHit ? " terrain" : "")
                             << (inside ? " inside-static" : "");
            object.moveBy(Misc::Convert::toOsg(offset));
        }
    }

    void PhysicsSystem::remove(const MWWorld::Ptr& ptr)
    {
        if (auto foundObject = mObjects.find(ptr.mRef); foundObject != mObjects.end())
        {
            mAnimatedObjects.erase(foundObject->second.get());
            if (foundObject->second->isDynamic())
                std::erase(mDynamicObjects, foundObject->second);

            mObjects.erase(foundObject);
        }
        else if (auto foundActor = mActors.find(ptr.mRef); foundActor != mActors.end())
        {
            mActors.erase(foundActor);
            mRagdolls.erase(ptr.mRef);
        }
    }

    void PhysicsSystem::removeProjectile(const int projectileId)
    {
        ProjectileMap::iterator foundProjectile = mProjectiles.find(projectileId);
        if (foundProjectile != mProjectiles.end())
            mProjectiles.erase(foundProjectile);
    }

    void PhysicsSystem::updatePtr(const MWWorld::Ptr& old, const MWWorld::Ptr& updated)
    {
        if (auto foundObject = mObjects.find(old.mRef); foundObject != mObjects.end())
            foundObject->second->updatePtr(updated);
        else if (auto foundActor = mActors.find(old.mRef); foundActor != mActors.end())
            foundActor->second->updatePtr(updated);
        if (auto foundRagdoll = mRagdolls.find(old.mRef); foundRagdoll != mRagdolls.end())
            foundRagdoll->second->updatePtr(updated);

        for (auto& [_, actor] : mActors)
        {
            if (actor->getStandingOnPtr() == old)
                actor->setStandingOnPtr(updated);
        }

        for (auto& [_, projectile] : mProjectiles)
        {
            if (projectile->getCaster() == old)
                projectile->setCaster(updated);
        }
    }

    Actor* PhysicsSystem::getActor(const MWWorld::Ptr& ptr)
    {
        ActorMap::iterator found = mActors.find(ptr.mRef);
        if (found != mActors.end())
            return found->second.get();
        return nullptr;
    }

    const Actor* PhysicsSystem::getActor(const MWWorld::ConstPtr& ptr) const
    {
        ActorMap::const_iterator found = mActors.find(ptr.mRef);
        if (found != mActors.end())
            return found->second.get();
        return nullptr;
    }

    const Object* PhysicsSystem::getObject(const MWWorld::ConstPtr& ptr) const
    {
        ObjectMap::const_iterator found = mObjects.find(ptr.mRef);
        if (found != mObjects.end())
            return found->second.get();
        return nullptr;
    }

    Projectile* PhysicsSystem::getProjectile(int projectileId) const
    {
        ProjectileMap::const_iterator found = mProjectiles.find(projectileId);
        if (found != mProjectiles.end())
            return found->second.get();
        return nullptr;
    }

    void PhysicsSystem::updateScale(const MWWorld::Ptr& ptr)
    {
        if (auto foundObject = mObjects.find(ptr.mRef); foundObject != mObjects.end())
        {
            float scale = ptr.getCellRef().getScale();
            foundObject->second->setScale(scale);
            mTaskScheduler->updateSingleAabb(foundObject->second);
        }
        else if (auto foundActor = mActors.find(ptr.mRef); foundActor != mActors.end())
        {
            foundActor->second->updateScale();
            mTaskScheduler->updateSingleAabb(foundActor->second);
        }
    }

    void PhysicsSystem::updateRotation(const MWWorld::Ptr& ptr, osg::Quat rotate)
    {
        if (auto foundObject = mObjects.find(ptr.mRef); foundObject != mObjects.end())
        {
            // The simulation is the source of this rotation; don't feed it back.
            if (mMovingDynamicObjects && foundObject->second->isDynamic())
                return;
            foundObject->second->setRotation(rotate);
            mTaskScheduler->updateSingleAabb(foundObject->second);
        }
        else if (auto foundActor = mActors.find(ptr.mRef); foundActor != mActors.end())
        {
            if (!foundActor->second->isRotationallyInvariant())
            {
                foundActor->second->setRotation(rotate);
                mTaskScheduler->updateSingleAabb(foundActor->second);
            }
        }
    }

    void PhysicsSystem::updatePosition(const MWWorld::Ptr& ptr)
    {
        if (auto foundObject = mObjects.find(ptr.mRef); foundObject != mObjects.end())
        {
            if (mMovingDynamicObjects && foundObject->second->isDynamic())
                return;
            foundObject->second->updatePosition();
            if (foundObject->second->isDynamic())
                placeOnSurface(*foundObject->second);
            mTaskScheduler->updateSingleAabb(foundObject->second);
        }
        else if (auto foundActor = mActors.find(ptr.mRef); foundActor != mActors.end())
        {
            foundActor->second->updatePosition();
            mTaskScheduler->updateSingleAabb(foundActor->second, true);
        }
    }

    void PhysicsSystem::addActor(const MWWorld::Ptr& ptr, VFS::Path::NormalizedView mesh)
    {
        const VFS::Path::Normalized animationMesh
            = Misc::ResourceHelpers::correctActorModelPath(mesh, mResourceSystem->getVFS());
        std::shared_ptr<const Resource::BulletShape> shape = mShapeManager->getShape(animationMesh);

        // Try to get shape from basic model as fallback for creatures
        if (!ptr.getClass().isNpc() && shape && shape->mCollisionBox.mExtents.length2() == 0)
        {
            if (animationMesh != mesh)
            {
                shape = mShapeManager->getShape(mesh);
            }
        }

        if (!shape)
            return;

        // check if Actor should spawn above water
        const MWMechanics::MagicEffects& effects = ptr.getClass().getCreatureStats(ptr).getMagicEffects();
        const bool canWaterWalk = effects.getOrDefault(ESM::MagicEffect::WaterWalking).getMagnitude() > 0;

        auto actor = std::make_shared<Actor>(
            ptr, *shape, mTaskScheduler.get(), canWaterWalk, Settings::game().mActorCollisionShapeType);

        mActors.emplace(ptr.mRef, std::move(actor));
    }

    int PhysicsSystem::addProjectile(
        const MWWorld::Ptr& caster, const osg::Vec3f& position, VFS::Path::NormalizedView mesh, bool computeRadius)
    {
        std::shared_ptr<Resource::BulletShapeInstance> shapeInstance = mShapeManager->getInstance(mesh);
        assert(shapeInstance);
        float radius = computeRadius ? shapeInstance->mCollisionBox.mExtents.length() / 2.f : 1.f;

        mProjectileId++;

        auto projectile = std::make_shared<Projectile>(caster, position, radius, mTaskScheduler.get(), this);
        mProjectiles.emplace(mProjectileId, std::move(projectile));

        return mProjectileId;
    }

    void PhysicsSystem::setCaster(int projectileId, const MWWorld::Ptr& caster)
    {
        const auto foundProjectile = mProjectiles.find(projectileId);
        assert(foundProjectile != mProjectiles.end());
        auto* projectile = foundProjectile->second.get();

        projectile->setCaster(caster);
    }

    bool PhysicsSystem::toggleCollisionMode()
    {
        ActorMap::iterator found = mActors.find(MWMechanics::getPlayer().mRef);
        if (found != mActors.end())
        {
            bool cmode = found->second->getCollisionMode();
            cmode = !cmode;
            found->second->enableCollisionMode(cmode);
            // NB: Collision body isn't disabled for vanilla TCL compatibility
            return cmode;
        }

        return false;
    }

    void PhysicsSystem::queueObjectMovement(const MWWorld::Ptr& ptr, const osg::Vec3f& velocity)
    {
        ActorMap::iterator found = mActors.find(ptr.mRef);
        if (found != mActors.end())
            found->second->setVelocity(velocity);
    }

    void PhysicsSystem::clearQueuedMovement()
    {
        for (const auto& [_, actor] : mActors)
        {
            actor->setVelocity(osg::Vec3f());
            actor->setInertialForce(osg::Vec3f());
        }
    }

    void PhysicsSystem::prepareSimulation(bool willSimulate, std::vector<Simulation>& simulations)
    {
        assert(simulations.empty());
        simulations.reserve(mActors.size() + mProjectiles.size());
        const MWBase::World* world = MWBase::Environment::get().getWorld();
        for (const auto& [ref, physicActor] : mActors)
        {
            if (!physicActor->isActive())
                continue;

            auto ptr = physicActor->getPtr();
            if (!ptr.getClass().isMobile(ptr))
                continue;

            const MWWorld::CellStore& cell = *ptr.getCell();
            const auto& stats = ptr.getClass().getCreatureStats(ptr);
            const MWMechanics::MagicEffects& effects = stats.getMagicEffects();

            float waterlevel = -std::numeric_limits<float>::max();
            bool waterCollision = false;
            if (cell.getCell()->hasWater())
            {
                waterlevel = cell.getWaterLevel();
                if (physicActor->getCollisionMode())
                    waterCollision = effects.getOrDefault(ESM::MagicEffect::WaterWalking).getMagnitude();
            }

            physicActor->setCanWaterWalk(waterCollision);

            // Slow fall reduces fall speed by a factor of (effect magnitude / 200)
            const float slowFall
                = 1.f - std::clamp(effects.getOrDefault(ESM::MagicEffect::SlowFall).getMagnitude() * 0.005f, 0.f, 1.f);
            const bool isPlayer = ptr == world->getPlayerConstPtr();
            const bool godmode = isPlayer && world->getGodModeState();
            const bool inert = stats.isDead()
                || (!godmode && stats.getMagicEffects().getOrDefault(ESM::MagicEffect::Paralyze).getModifier() > 0);

            simulations.emplace_back(ActorSimulation{
                physicActor, ActorFrameData{ *physicActor, inert, waterCollision, slowFall, waterlevel, isPlayer } });

            // if the simulation will run, a jump request will be fulfilled. Update mechanics accordingly.
            if (willSimulate)
                handleJump(ptr);
        }

        for (const auto& [id, projectile] : mProjectiles)
        {
            simulations.emplace_back(ProjectileSimulation{ projectile, ProjectileFrameData{ *projectile } });
        }
    }

    void PhysicsSystem::stepSimulation(
        float dt, bool skipSimulation, osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats)
    {
        for (auto& [animatedObject, changed] : mAnimatedObjects)
        {
            if (animatedObject->animateCollisionShapes())
            {
                auto obj = mObjects.find(animatedObject->getPtr().mRef);
                assert(obj != mObjects.end());
                mTaskScheduler->updateSingleAabb(obj->second);
                changed = true;
            }
            else
            {
                changed = false;
            }
        }
        for (auto& [_, object] : mObjects)
            object->resetCollisions();

#ifndef BT_NO_PROFILE
        CProfileManager::Reset();
        CProfileManager::Increment_Frame_Counter();
#endif

        mTimeAccum += dt;

        if (skipSimulation)
            mTaskScheduler->resetSimulation(mActors);
        else
        {
            std::vector<Simulation>& simulations = mSimulations[mSimulationsCounter++ % mSimulations.size()];
            prepareSimulation(mTimeAccum >= mPhysicsDt, simulations);
            // modifies mTimeAccum
            mTaskScheduler->applyQueuedMovements(mTimeAccum, simulations, frameStart, frameNumber, stats);
        }
    }

    std::vector<MWWorld::Ptr> PhysicsSystem::moveActors()
    {
        auto* player = getActor(MWMechanics::getPlayer());
        const auto world = MWBase::Environment::get().getWorld();

        // copy new ptr position in temporary vector. player is handled separately as its movement might change active
        // cell.
        mActorsPositions.clear();
        if (!mActors.empty())
            mActorsPositions.reserve(mActors.size() - 1);
        std::vector<std::pair<MWWorld::Ptr, osg::Vec3f>> bodyPositions;
        for (const auto& [ptr, physicActor] : mActors)
        {
            if (physicActor.get() == player)
                continue;
            if (const auto ragdoll = mRagdolls.find(ptr); ragdoll != mRagdolls.end())
            {
                // The body carries the actor along, so it is where the body is (for its cell, and after loading).
                // As its model was posed this frame (see getRagdollBonePoses).
                const auto& poses = ragdoll->second->getLastBonePoses();
                const MWWorld::Ptr actorPtr = physicActor->getPtr();
                if (poses.size() == 1 && poses.front().first.empty())
                {
                    // In one piece: the whole model is the body.
                    osg::Vec3f translation;
                    osg::Quat rotation;
                    osg::Vec3f scale;
                    osg::Quat scaleOrientation;
                    poses.front().second.decompose(translation, rotation, scale, scaleOrientation);
                    if (SceneUtil::PositionAttitudeTransform* base = actorPtr.getRefData().getBaseNode())
                        base->setAttitude(rotation);
                    bodyPositions.emplace_back(actorPtr, translation);
                }
                else if (const std::optional<osg::Vec3f> position
                         = ragdoll->second->getActorPosition(actorPtr.getRefData().getPosition().asVec3());
                         position && (*position - actorPtr.getRefData().getPosition().asVec3()).length2() > 1.f)
                {
                    // Limp, or in pieces: the actor follows the main part; its bones are set where they are.
                    bodyPositions.emplace_back(actorPtr, *position);
                }
                continue;
            }
            mActorsPositions.emplace_back(physicActor->getPtr(), physicActor->getSimulationPosition());
        }

        for (const auto& [ptr, pos] : mActorsPositions)
            world->moveObject(ptr, pos, false, false);
        std::vector<MWWorld::Ptr> moved;
        for (const auto& [ptr, pos] : bodyPositions)
            moved.push_back(world->moveObject(ptr, pos, true, false));

        if (player != nullptr)
            world->moveObject(player->getPtr(), player->getSimulationPosition(), false, false);

        moveDynamicObjects();
        return moved;
    }

    void PhysicsSystem::moveDynamicObjects()
    {
        if (mDynamicObjects.empty())
            return;

        const auto world = MWBase::Environment::get().getWorld();
        // Moving an object out of the active cells removes it from mDynamicObjects; iterate over a copy.
        const std::vector<std::shared_ptr<Object>> objects = mDynamicObjects;
        mMovingDynamicObjects = true;
        for (const auto& object : objects)
        {
            const float heightBefore = static_cast<float>(object->getTransform().getOrigin().z());
            const auto transform = object->takeSimulatedTransform();
            if (!transform)
                continue;

            if (mWaterEnabled && heightBefore > mWaterHeight && transform->first.z() <= mWaterHeight)
            {
                // Simulation steps are 1/mPhysicsDt per second.
                const float speed = (heightBefore - transform->first.z()) / mPhysicsDt;
                world->objectEnteredWater(
                    osg::Vec3f(transform->first.x(), transform->first.y(), mWaterHeight), speed);
            }

            // Safety net: something that slipped through the ground would fall forever. Put it back on top.
            const MWWorld::Ptr current = object->getPtr();
            const MWWorld::CellStore* cell = current.getCell();
            const osg::Vec3f& position = transform->first;
            std::optional<osg::Vec3f> rescue;
            if (cell->isExterior())
            {
                const float terrain = world->getTerrainHeightAt(position, cell->getCell()->getWorldSpace());
                if (position.z() < terrain - 100.f)
                    rescue = osg::Vec3f(position.x(), position.y(), terrain);
            }
            else if (position.z() < object->getPlacedPosition().z() - 3000.f)
                rescue = object->getPlacedPosition();
            if (rescue)
            {
                Log(Debug::Verbose) << "[physics] rescued " << current.getCellRef().getRefId() << " from z "
                                 << position.z();
                world->moveObject(current, *rescue, false, false);
                object->updatePosition();
                placeOnSurface(*object);
                mTaskScheduler->updateSingleAabb(object);
                continue;
            }

            const MWWorld::Ptr ptr = world->moveObject(current, position, false, false);
            world->rotateObject(ptr, toEsmRotation(transform->second), MWBase::RotationFlag_none);
        }
        mMovingDynamicObjects = false;
    }

    void PhysicsSystem::updateAnimatedCollisionShape(const MWWorld::Ptr& object)
    {
        ObjectMap::iterator found = mObjects.find(object.mRef);
        if (found != mObjects.end())
            if (found->second->animateCollisionShapes())
                mTaskScheduler->updateSingleAabb(found->second);
    }

    void PhysicsSystem::debugDraw()
    {
        if (mDebugDrawEnabled)
            mTaskScheduler->debugDraw();
    }

    bool PhysicsSystem::isActorStandingOn(const MWWorld::Ptr& actor, const MWWorld::ConstPtr& object) const
    {
        const auto physActor = mActors.find(actor.mRef);
        if (physActor != mActors.end())
            return physActor->second->getStandingOnPtr() == object;
        return false;
    }

    void PhysicsSystem::getActorsStandingOn(const MWWorld::ConstPtr& object, std::vector<MWWorld::Ptr>& out) const
    {
        for (const auto& [_, actor] : mActors)
        {
            if (actor->getStandingOnPtr() == object)
                out.emplace_back(actor->getPtr());
        }
    }

    bool PhysicsSystem::isObjectCollidingWith(const MWWorld::ConstPtr& object, ScriptedCollisionType type) const
    {
        auto found = mObjects.find(object.mRef);
        if (found != mObjects.end())
            return found->second->collidedWith(type);
        return false;
    }

    void PhysicsSystem::getActorsCollidingWith(const MWWorld::ConstPtr& object, std::vector<MWWorld::Ptr>& out) const
    {
        std::vector<MWWorld::Ptr> collisions = getCollisions(object, CollisionType_World, CollisionType_Actor);
        out.insert(out.end(), collisions.begin(), collisions.end());
    }

    void PhysicsSystem::disableWater()
    {
        if (mWaterEnabled)
        {
            mWaterEnabled = false;
            updateWater();
        }
    }

    void PhysicsSystem::enableWater(float height)
    {
        if (!mWaterEnabled || mWaterHeight != height)
        {
            mWaterEnabled = true;
            mWaterHeight = height;
            updateWater();
        }
    }

    void PhysicsSystem::setWaterHeight(float height)
    {
        if (mWaterHeight != height)
        {
            mWaterHeight = height;
            updateWater();
        }
    }

    void PhysicsSystem::updateWater()
    {
        mTaskScheduler->setWaterHeight(mWaterEnabled ? std::optional(mWaterHeight) : std::nullopt);

        if (mWaterCollisionObject)
        {
            mTaskScheduler->removeCollisionObject(mWaterCollisionObject.get());
        }

        if (!mWaterEnabled)
        {
            mWaterCollisionObject.reset();
            return;
        }

        mWaterCollisionObject = std::make_unique<btCollisionObject>();
        mWaterCollisionShape = std::make_unique<btStaticPlaneShape>(btVector3(0, 0, 1), mWaterHeight);
        mWaterCollisionObject->setCollisionShape(mWaterCollisionShape.get());
        mWaterCollisionObject->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
        // Static, so never "awake": the dynamics step skips pairs where both objects sleep.
        mWaterCollisionObject->setActivationState(ISLAND_SLEEPING);
        mTaskScheduler->addCollisionObject(
            mWaterCollisionObject.get(), CollisionType_Water, CollisionType_Actor | CollisionType_Projectile);
    }

    bool PhysicsSystem::isAreaOccupiedByOtherActor(
        const MWWorld::LiveCellRefBase* actor, const osg::Vec3f& position, const float radius) const
    {
        const btCollisionObject* ignoredObject = nullptr;
        if (const auto it = mActors.find(actor); it != mActors.end())
            ignoredObject = it->second->getCollisionObject();
        const btVector3 bulletPosition = Misc::Convert::toBullet(position);
        const btVector3 aabbMin = bulletPosition - btVector3(radius, radius, radius);
        const btVector3 aabbMax = bulletPosition + btVector3(radius, radius, radius);
        const int mask = MWPhysics::CollisionType_Actor;
        const int group = MWPhysics::CollisionType_AnyPhysical;
        HasSphereCollisionCallback callback(bulletPosition, radius, mask, group, ignoredObject);
        mTaskScheduler->aabbTest(aabbMin, aabbMax, callback);
        return callback.getResult();
    }

    void PhysicsSystem::reportStats(unsigned int frameNumber, osg::Stats& stats) const
    {
        stats.setAttribute(frameNumber, "Physics Actors", static_cast<double>(mActors.size()));
        stats.setAttribute(frameNumber, "Physics Objects", static_cast<double>(mObjects.size()));
        stats.setAttribute(frameNumber, "Physics Projectiles", static_cast<double>(mProjectiles.size()));
        stats.setAttribute(frameNumber, "Physics HeightFields", static_cast<double>(mHeightFields.size()));
    }

    void PhysicsSystem::reportCollision(const btVector3& position, const btVector3& normal)
    {
        if (mDebugDrawEnabled)
            mDebugDrawer->addCollision(position, normal);
    }

    ActorFrameData::ActorFrameData(
        Actor& actor, bool inert, bool waterCollision, float slowFall, float waterlevel, bool isPlayer)
        : mPosition()
        , mStandingOn(nullptr)
        , mIsOnGround(actor.getOnGround())
        , mIsOnSlope(actor.getOnSlope())
        , mWalkingOnWater(false)
        , mInert(inert)
        , mCollisionObject(actor.getCollisionObject())
        , mSwimLevel(waterlevel
              - (actor.getRenderingHalfExtents().z() * 2
                  * MWBase::Environment::get()
                        .getESMStore()
                        ->get<ESM::GameSetting>()
                        .find("fSwimHeightScale")
                        ->mValue.getFloat()))
        , mSlowFall(slowFall)
        , mRotation()
        , mMovement(actor.velocity())
        , mWaterlevel(waterlevel)
        , mHalfExtentsZ(actor.getHalfExtents().z())
        , mOldHeight(0)
        , mStuckFrames(0)
        , mFlying(MWBase::Environment::get().getWorld()->isFlying(actor.getPtr()))
        , mWasOnGround(actor.getOnGround())
        , mIsAquatic(actor.getPtr().getClass().isPureWaterCreature(actor.getPtr()))
        , mWaterCollision(waterCollision)
        , mSkipCollisionDetection(!actor.getCollisionMode())
        , mIsPlayer(isPlayer)
    {
    }

    ProjectileFrameData::ProjectileFrameData(Projectile& projectile)
        : mPosition(projectile.getPosition())
        , mMovement(projectile.velocity())
        , mCaster(projectile.getCasterCollisionObject())
        , mCollisionObject(projectile.getCollisionObject())
        , mProjectile(&projectile)
    {
    }

    WorldFrameData::WorldFrameData()
        : mIsInStorm(MWBase::Environment::get().getWorld()->isInStorm())
        , mStormDirection(MWBase::Environment::get().getWorld()->getStormDirection())
    {
    }

    LOSRequest::LOSRequest(const std::weak_ptr<Actor>& a1, const std::weak_ptr<Actor>& a2)
        : mResult(false)
        , mStale(false)
        , mAge(0)
    {
        // we use raw actor pointer pair to uniquely identify request
        // sort the pointer value in ascending order to not duplicate equivalent requests, eg. getLOS(A, B) and
        // getLOS(B, A)
        auto* raw1 = a1.lock().get();
        auto* raw2 = a2.lock().get();
        assert(raw1 != raw2);
        if (raw1 < raw2)
        {
            mActors = { a1, a2 };
            mRawActors = { raw1, raw2 };
        }
        else
        {
            mActors = { a2, a1 };
            mRawActors = { raw2, raw1 };
        }
    }

    bool operator==(const LOSRequest& lhs, const LOSRequest& rhs) noexcept
    {
        return lhs.mRawActors == rhs.mRawActors;
    }
}
