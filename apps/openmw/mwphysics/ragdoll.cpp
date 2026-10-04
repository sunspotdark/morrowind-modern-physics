#include "ragdoll.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string_view>

#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#include <BulletDynamics/Dynamics/btActionInterface.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btMotionState.h>

#include <osg/Quat>

#include <components/misc/convert.hpp>

#include "collisiontype.hpp"
#include "mtphysics.hpp"

namespace MWPhysics
{
    // Receives body transforms from Bullet on the physics thread; the main thread reads them for the bones.
    class RagdollMotionState final : public btMotionState
    {
    public:
        explicit RagdollMotionState(const btTransform& transform)
            : mTransform(transform)
            , mPreviousTransform(transform)
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
            mPreviousTransform = mTransform;
            mTransform = transform;
        }

        /// Between the last two physics steps: 0 is the one before, 1 the latest.
        btTransform getInterpolatedTransform(btScalar factor) const
        {
            std::lock_guard lock(mMutex);
            return btTransform(mPreviousTransform.getRotation().slerp(mTransform.getRotation(), factor),
                mPreviousTransform.getOrigin().lerp(mTransform.getOrigin(), factor));
        }

    private:
        mutable std::mutex mMutex;
        btTransform mTransform;
        btTransform mPreviousTransform;
    };

    // Friction in the joints: each step, some of the way jointed parts turn relative to each other is taken out.
    // Without it a body collapses like a sack of liquid; with it, limbs still flop but settle instead of sloshing.
    class JointFriction final : public btActionInterface
    {
    public:
        std::vector<std::pair<btRigidBody*, btRigidBody*>> mPairs;

        void updateAction(btCollisionWorld*, btScalar timeStep) override
        {
            constexpr btScalar rate = 14; // per second: about a fifth of the turning gone each step at 60 Hz
            const btScalar fraction = 1 - std::exp(-rate * timeStep);
            for (const auto& [a, b] : mPairs)
            {
                if (!a->isActive() && !b->isActive())
                    continue;
                const btVector3 relative = b->getAngularVelocity() - a->getAngularVelocity();
                if (relative.length2() < 1e-4f)
                    continue;
                // The lighter part gives more.
                const btScalar massA = a->getInvMass() > 0 ? 1 / a->getInvMass() : 0;
                const btScalar massB = b->getInvMass() > 0 ? 1 / b->getInvMass() : 0;
                if (massA + massB <= 0)
                    continue;
                const btVector3 change = relative * fraction;
                a->setAngularVelocity(a->getAngularVelocity() + change * (massB / (massA + massB)));
                b->setAngularVelocity(b->getAngularVelocity() - change * (massA / (massA + massB)));
            }
        }

        void debugDraw(btIDebugDraw*) override {}
    };

    struct Ragdoll::Part final : public PtrHolder
    {
        explicit Part(const MWWorld::Ptr& ptr)
            : PtrHolder(ptr, osg::Vec3f())
        {
        }

        std::string mBone;
        std::unique_ptr<btCollisionShape> mShape;
        std::unique_ptr<RagdollMotionState> mMotionState;
        btRigidBody* mBody = nullptr; // owned by mCollisionObject

        void setBody(std::unique_ptr<btRigidBody> body)
        {
            mBody = body.get();
            mCollisionObject = std::move(body);
        }
        btTransform mBoneFromBody; // the bone's transform in the body's frame (without scale)
        osg::Vec3f mBoneScale;
        osg::Vec3f mBoxHalfExtents; // for a single box body
    };

    namespace
    {
        struct PartDef
        {
            std::string_view mBone;
            std::string_view mEnd; // where the part ends; empty for the head
            float mRadius;
            float mMass;
            int mParent; // index in sPartDefs, -1 for none
            // Joint limits (degrees): swing around two axes, twist around the bone
            float mSwing1;
            float mSwing2;
            float mTwist;
        };

        // A human-sized body (about 128 units tall). Knees and elbows swing mostly one way, so one swing span
        // is kept small; shoulders and hips are loose.
        constexpr PartDef sPartDefs[] = {
            { "Bip01 Pelvis", "Bip01 Spine1", 11.f, 12.f, -1, 0, 0, 0 },
            { "Bip01 Spine1", "Bip01 Neck", 12.f, 15.f, 0, 25, 25, 15 },
            { "Bip01 Head", "", 8.f, 5.f, 1, 40, 40, 40 },
            { "Bip01 L UpperArm", "Bip01 L Forearm", 4.5f, 3.f, 1, 80, 80, 40 },
            { "Bip01 L Forearm", "Bip01 L Hand", 4.f, 2.f, 3, 70, 10, 20 },
            { "Bip01 R UpperArm", "Bip01 R Forearm", 4.5f, 3.f, 1, 80, 80, 40 },
            { "Bip01 R Forearm", "Bip01 R Hand", 4.f, 2.f, 5, 70, 10, 20 },
            { "Bip01 L Thigh", "Bip01 L Calf", 6.5f, 7.f, 0, 60, 40, 15 },
            { "Bip01 L Calf", "Bip01 L Foot", 5.f, 4.f, 7, 70, 10, 10 },
            { "Bip01 R Thigh", "Bip01 R Calf", 6.5f, 7.f, 0, 60, 40, 15 },
            { "Bip01 R Calf", "Bip01 R Foot", 5.f, 4.f, 9, 70, 10, 10 },
        };

        btTransform withoutScale(const osg::Matrixf& matrix, osg::Vec3f& scale)
        {
            osg::Vec3f translation;
            osg::Quat rotation;
            osg::Quat scaleOrientation;
            matrix.decompose(translation, rotation, scale, scaleOrientation);
            return btTransform(Misc::Convert::toBullet(rotation), Misc::Convert::toBullet(translation));
        }
    }

    const std::vector<std::string>& Ragdoll::getRequiredBones()
    {
        static const std::vector<std::string> bones = { "Bip01 Pelvis", "Bip01 Spine1", "Bip01 Neck", "Bip01 Head",
            "Bip01 L UpperArm", "Bip01 L Forearm", "Bip01 L Hand", "Bip01 R UpperArm", "Bip01 R Forearm",
            "Bip01 R Hand", "Bip01 L Thigh", "Bip01 L Calf", "Bip01 L Foot", "Bip01 R Thigh", "Bip01 R Calf",
            "Bip01 R Foot" };
        return bones;
    }

    Ragdoll::Ragdoll(const MWWorld::Ptr& actor, const BoneMatrices& bones, const osg::Vec3f& velocity,
        const osg::Vec3f& kick, PhysicsTaskScheduler* scheduler)
        : mTaskScheduler(scheduler)
    {
        const auto position = [&](std::string_view bone) { return bones.find(bone)->second.getTrans(); };

        for (const PartDef& def : sPartDefs)
        {
            auto part = std::make_shared<Part>(actor);
            part->mBone = def.mBone;

            const osg::Vec3f start = position(def.mBone);
            osg::Vec3f end;
            if (!def.mEnd.empty())
                end = position(def.mEnd);
            else
            {
                // The head goes on from the neck.
                osg::Vec3f up = start - position("Bip01 Neck");
                if (up.normalize() < 1e-3f)
                    up = osg::Vec3f(0, 0, 1);
                end = start + up * 14.f;
            }
            btVector3 axis = Misc::Convert::toBullet(end - start);
            const btScalar length = axis.length();
            axis = length > 1e-3f ? axis / length : btVector3(0, 0, 1);

            // A capsule along the bone (capsules lie along their Y axis).
            const btTransform bodyWorld(
                shortestArcQuat(btVector3(0, 1, 0), axis), Misc::Convert::toBullet((start + end) * 0.5f));
            part->mShape = std::make_unique<btCapsuleShape>(
                def.mRadius, std::max(length - 2 * def.mRadius, btScalar(0.1)));
            part->mBoneFromBody
                = bodyWorld.inverse() * withoutScale(bones.find(def.mBone)->second, part->mBoneScale);

            makeBody(*part, bodyWorld, def.mMass, def.mRadius);

            // Falling as it was moving, the upper body thrown back by the killing blow.
            btVector3 partVelocity = Misc::Convert::toBullet(velocity);
            if (def.mBone == "Bip01 Spine1" || def.mBone == "Bip01 Head")
                partVelocity += Misc::Convert::toBullet(kick);
            else if (def.mBone == "Bip01 Pelvis")
                partVelocity += Misc::Convert::toBullet(kick) * 0.6f;
            part->mBody->setLinearVelocity(partVelocity);

            mParts.push_back(std::move(part));
        }
        addToWorld();

        // Joints where each part meets its parent, twisting around the part's own bone.
        for (size_t i = 0; i < mParts.size(); ++i)
        {
            const PartDef& def = sPartDefs[i];
            if (def.mParent >= 0)
                addJoint(def.mParent, i, position(def.mBone), def.mSwing1, def.mSwing2, def.mTwist);
        }
        addJointFriction();
    }

    Ragdoll::Ragdoll(const MWWorld::Ptr& actor, const btTransform& bodyWorld, const osg::Vec3f& halfExtents,
        float mass, const osg::Matrixf& baseWorld, PhysicsTaskScheduler* scheduler)
        : mTaskScheduler(scheduler)
    {
        auto part = std::make_shared<Part>(actor);
        part->mBoxHalfExtents = halfExtents;
        part->mShape = std::make_unique<btBoxShape>(Misc::Convert::toBullet(halfExtents));
        part->mBoneFromBody = bodyWorld.inverse() * withoutScale(baseWorld, part->mBoneScale);
        makeBody(*part, bodyWorld, mass, 0);
        mParts.push_back(std::move(part));
        addToWorld();
    }

    std::unique_ptr<Ragdoll> Ragdoll::fromSkeleton(const MWWorld::Ptr& actor, const std::vector<SkinnedBone>& bones,
        const osg::Vec3f& velocity, const osg::Vec3f& kick, PhysicsTaskScheduler* scheduler)
    {
        // A part for each bone that moves a fair share of the mesh (legs, body, head, tail...), at most a body's
        // worth; smaller ones (toes, jaws, tail tips) just go along with the part above them.
        float largest = 0;
        for (const SkinnedBone& bone : bones)
            largest = std::max(largest, bone.mRadius);
        std::vector<size_t> chosen;
        for (size_t i = 0; i < bones.size(); ++i)
            if (bones[i].mRadius >= largest * 0.2f && bones[i].mRadius > 1.f)
                chosen.push_back(i);
        constexpr size_t maxParts = 12;
        if (chosen.size() > maxParts)
        {
            std::vector<size_t> bySize = chosen;
            std::nth_element(bySize.begin(), bySize.begin() + maxParts - 1, bySize.end(),
                [&](size_t a, size_t b) { return bones[a].mRadius > bones[b].mRadius; });
            const float cutoff = bones[bySize[maxParts - 1]].mRadius;
            std::erase_if(chosen, [&](size_t i) { return bones[i].mRadius < cutoff; });
            chosen.resize(std::min(chosen.size(), maxParts));
        }
        if (chosen.size() < 2)
            return nullptr;

        std::unique_ptr<Ragdoll> ragdoll(new Ragdoll(scheduler));
        std::vector<int> partOfBone(bones.size(), -1);
        for (size_t i : chosen)
        {
            const SkinnedBone& bone = bones[i];
            auto part = std::make_shared<Part>(actor);
            part->mBone = bone.mName;
            const btTransform boneWorld = withoutScale(bone.mWorld, part->mBoneScale);
            const float scale = part->mBoneScale.x();

            // The bone's share of the mesh lies around the center of its bounds. Away from the bone it is a
            // limb, as long as twice that and as thick as the bounds allow; around the bone, a lump.
            const btVector3 center = boneWorld * Misc::Convert::toBullet(bone.mCenter * scale);
            const float radius = bone.mRadius * scale;
            const float half = bone.mCenter.length() * scale;
            btTransform bodyWorld;
            float thickness;
            if (half > radius * 0.35f)
            {
                const btVector3 axis = (center - boneWorld.getOrigin()).normalized();
                thickness = std::clamp(std::sqrt(std::max(radius * radius - half * half, 0.f)) * 0.9f,
                    std::max(radius * 0.2f, 1.5f), radius);
                part->mShape = std::make_unique<btCapsuleShape>(thickness, std::max(2 * half - 2 * thickness, 0.1f));
                bodyWorld = btTransform(shortestArcQuat(btVector3(0, 1, 0), axis), center);
            }
            else
            {
                thickness = std::max(radius * 0.7f, 1.5f);
                part->mShape = std::make_unique<btSphereShape>(thickness);
                bodyWorld = btTransform(boneWorld.getRotation(), center);
            }
            part->mBoneFromBody = bodyWorld.inverse() * boneWorld;

            // Flesh is about as dense as water (~70 units to a meter).
            btVector3 aabbMin;
            btVector3 aabbMax;
            part->mShape->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
            const btVector3 size = aabbMax - aabbMin;
            const float volume = static_cast<float>(size.x() * size.y() * size.z()) * 0.6f / (70.f * 70.f * 70.f);
            makeBody(*part, bodyWorld, std::clamp(volume * 1000.f, 0.3f, 200.f), thickness);
            part->mBody->setLinearVelocity(Misc::Convert::toBullet(velocity + kick * 0.7f));

            partOfBone[i] = static_cast<int>(ragdoll->mParts.size());
            ragdoll->mParts.push_back(std::move(part));
        }
        ragdoll->addToWorld();

        // Joined to the part above; any parts with none (bits not under the main body) to the biggest.
        const auto parentPart = [&](size_t bone) {
            for (int parent = bones[bone].mParent; parent >= 0; parent = bones[parent].mParent)
                if (partOfBone[parent] >= 0)
                    return partOfBone[parent];
            return -1;
        };
        int root = 0;
        for (size_t i = 0; i < chosen.size(); ++i)
            if (parentPart(chosen[i]) < 0 && bones[chosen[i]].mRadius > bones[chosen[root]].mRadius)
                root = static_cast<int>(i);
        for (size_t i = 0; i < chosen.size(); ++i)
        {
            int parent = parentPart(chosen[i]);
            if (parent < 0 && static_cast<int>(i) != root)
                parent = root;
            if (parent >= 0)
                ragdoll->addJoint(parent, i, bones[chosen[i]].mWorld.getTrans(), 30, 30, 15);
        }
        ragdoll->addJointFriction();
        return ragdoll;
    }

    std::unique_ptr<Ragdoll> Ragdoll::fromRigidPieces(const MWWorld::Ptr& actor, const std::vector<RigidPiece>& pieces,
        const osg::Vec3f& velocity, const osg::Vec3f& kick, PhysicsTaskScheduler* scheduler)
    {
        // The bigger pieces come loose (legs, shell, head...), at most a handful; the rest stay on them.
        const auto size = [&](size_t i) { return (pieces[i].mMax - pieces[i].mMin).length() * 0.5f; };
        float largest = 0;
        for (size_t i = 0; i < pieces.size(); ++i)
            largest = std::max(largest, size(i));
        std::vector<size_t> chosen;
        for (size_t i = 0; i < pieces.size(); ++i)
            if (size(i) >= largest * 0.12f && size(i) > 1.f)
                chosen.push_back(i);
        constexpr size_t maxParts = 16;
        if (chosen.size() > maxParts)
        {
            std::vector<size_t> bySize = chosen;
            std::nth_element(bySize.begin(), bySize.begin() + maxParts - 1, bySize.end(),
                [&](size_t a, size_t b) { return size(a) > size(b); });
            const float cutoff = size(bySize[maxParts - 1]);
            std::erase_if(chosen, [&](size_t i) { return size(i) < cutoff; });
            chosen.resize(std::min(chosen.size(), maxParts));
        }
        if (chosen.size() < 2)
            return nullptr;

        std::unique_ptr<Ragdoll> ragdoll(new Ragdoll(scheduler));
        osg::Vec3f middle;
        std::vector<btTransform> bodies;
        for (size_t i : chosen)
        {
            const RigidPiece& piece = pieces[i];
            auto part = std::make_shared<Part>(actor);
            part->mBone = piece.mName;
            const btTransform nodeWorld = withoutScale(piece.mWorld, part->mBoneScale);
            const osg::Vec3f scale = part->mBoneScale;
            const osg::Vec3f center = (piece.mMin + piece.mMax) * 0.5f;
            osg::Vec3f half = (piece.mMax - piece.mMin) * 0.5f;
            half = osg::Vec3f(std::max(half.x() * scale.x(), 1.f), std::max(half.y() * scale.y(), 1.f),
                std::max(half.z() * scale.z(), 1.f));
            part->mBoxHalfExtents = half;
            part->mShape = std::make_unique<btBoxShape>(Misc::Convert::toBullet(half));
            const btTransform bodyWorld(nodeWorld.getRotation(),
                nodeWorld * Misc::Convert::toBullet(osg::Vec3f(
                    center.x() * scale.x(), center.y() * scale.y(), center.z() * scale.z())));
            part->mBoneFromBody = bodyWorld.inverse() * nodeWorld;
            // Shells and limbs: about half as dense as water, a box being roomier than what it holds.
            const float volume = 8.f * half.x() * half.y() * half.z() / (70.f * 70.f * 70.f);
            makeBody(*part, bodyWorld, std::clamp(volume * 500.f, 0.2f, 100.f),
                std::min({ half.x(), half.y(), half.z() }));
            middle += Misc::Convert::toOsg(bodyWorld.getOrigin());
            bodies.push_back(bodyWorld);
            ragdoll->mParts.push_back(std::move(part));
        }
        middle /= static_cast<float>(chosen.size());

        // Falling apart: the pieces spread a little from the middle, as the killing blow throws them.
        for (size_t i = 0; i < ragdoll->mParts.size(); ++i)
        {
            osg::Vec3f out = Misc::Convert::toOsg(bodies[i].getOrigin()) - middle;
            out.z() = 0;
            if (out.normalize() > 0)
                out *= 50.f;
            ragdoll->mParts[i]->mBody->setLinearVelocity(
                Misc::Convert::toBullet(velocity + kick * 0.7f + out + osg::Vec3f(0, 0, 40)));
        }
        ragdoll->addToWorld();
        return ragdoll;
    }

    Ragdoll::Ragdoll(PhysicsTaskScheduler* scheduler)
        : mTaskScheduler(scheduler)
    {
    }

    void Ragdoll::makeBody(Part& part, const btTransform& bodyWorld, float mass, float thickness)
    {
        btVector3 inertia(0, 0, 0);
        part.mShape->calculateLocalInertia(mass, inertia);
        part.mMotionState = std::make_unique<RagdollMotionState>(bodyWorld);
        btRigidBody::btRigidBodyConstructionInfo info(mass, part.mMotionState.get(), part.mShape.get(), inertia);
        info.m_friction = 0.8f;
        info.m_restitution = 0.05f;
        info.m_linearDamping = 0.15f;
        info.m_angularDamping = 0.7f;
        // In game units (~70 per meter), like the other simulated objects.
        info.m_linearSleepingThreshold = 4.f;
        info.m_angularSleepingThreshold = 1.f;
        auto body = std::make_unique<btRigidBody>(info);
        if (thickness > 0)
        {
            body->setCcdMotionThreshold(thickness * 0.5f);
            body->setCcdSweptSphereRadius(thickness * 0.8f);
            // Round parts would roll away down any slope. Rolling resistance has to grow with the size (holding
            // still takes a torque of weight x radius), so this holds them on slopes up to about 30 degrees.
            body->setRollingFriction(thickness);
            body->setSpinningFriction(thickness * 0.3f);
        }
        body->setUserPointer(&part);
        part.setBody(std::move(body));
    }

    void Ragdoll::addToWorld()
    {
        // The parts don't collide with each other: they overlap at the joints.
        for (size_t i = 0; i < mParts.size(); ++i)
            for (size_t j = i + 1; j < mParts.size(); ++j)
            {
                mParts[i]->mBody->setIgnoreCollisionCheck(mParts[j]->mBody, true);
                mParts[j]->mBody->setIgnoreCollisionCheck(mParts[i]->mBody, true);
            }
        for (const auto& part : mParts)
            mTaskScheduler->addRigidBody(part->mBody, CollisionType_Dynamic,
                CollisionType_DynamicSupport | CollisionType_Actor | CollisionType_Dynamic | CollisionType_Projectile);
    }

    void Ragdoll::addJoint(size_t parentIndex, size_t childIndex, const osg::Vec3f& position, float swing1,
        float swing2, float twist)
    {
        // Twisting around the child part's own length.
        btRigidBody& parent = *mParts[parentIndex]->mBody;
        btRigidBody& child = *mParts[childIndex]->mBody;
        const btTransform& childWorld = child.getWorldTransform();
        btVector3 childAxis = childWorld.getBasis() * btVector3(0, 1, 0);
        const btTransform jointWorld(
            shortestArcQuat(btVector3(1, 0, 0), childAxis), Misc::Convert::toBullet(position));
        auto joint = std::make_unique<btConeTwistConstraint>(
            parent, child, parent.getWorldTransform().inverse() * jointWorld, childWorld.inverse() * jointWorld);
        // Firm limits (little give, quickly corrected), so joints don't stretch.
        joint->setLimit(btRadians(swing1), btRadians(swing2), btRadians(twist), 0.9f, 0.6f, 1.f);
        mTaskScheduler->addConstraint(joint.get());
        mJoints.push_back(std::move(joint));
    }

    void Ragdoll::addJointFriction()
    {
        auto friction = std::make_unique<JointFriction>();
        for (const auto& joint : mJoints)
            friction->mPairs.emplace_back(&joint->getRigidBodyA(), &joint->getRigidBodyB());
        mJointFriction = std::move(friction);
        mTaskScheduler->addAction(mJointFriction.get());
    }

    float Ragdoll::getMass() const
    {
        float mass = 0;
        for (const auto& part : mParts)
            if (part->mBody->getInvMass() > 0)
                mass += static_cast<float>(1 / part->mBody->getInvMass());
        return mass;
    }

    std::optional<osg::Vec3f> Ragdoll::getGrabPoint(const osg::Vec3f& eye) const
    {
        if (mParts.size() != 1)
            return std::nullopt;
        const Part& part = *mParts.front();
        btTransform bodyWorld;
        part.mMotionState->getWorldTransform(bodyWorld);
        // The nearest point of the box, a little inside so the pin isn't right on the edge.
        btVector3 local = bodyWorld.inverse() * Misc::Convert::toBullet(eye);
        const osg::Vec3f& half = part.mBoxHalfExtents;
        local.setX(std::clamp<btScalar>(local.x(), -half.x() * 0.8f, half.x() * 0.8f));
        local.setY(std::clamp<btScalar>(local.y(), -half.y() * 0.8f, half.y() * 0.8f));
        local.setZ(std::clamp<btScalar>(local.z(), -half.z() * 0.8f, half.z() * 0.8f));
        return Misc::Convert::toOsg(bodyWorld * local);
    }

    Ragdoll::~Ragdoll()
    {
        if (mJointFriction != nullptr)
            mTaskScheduler->removeAction(mJointFriction.get());
        for (const auto& joint : mJoints)
            mTaskScheduler->removeConstraint(joint.get());
        for (const auto& part : mParts)
            mTaskScheduler->removeCollisionObject(part->mBody);
    }

    std::vector<std::pair<std::string, osg::Matrixf>> Ragdoll::getBonePoses(float interpolation) const
    {
        std::vector<std::pair<std::string, osg::Matrixf>> poses;
        poses.reserve(mParts.size());
        for (const auto& part : mParts)
        {
            const btTransform bodyWorld = part->mMotionState->getInterpolatedTransform(interpolation);
            const btTransform bone = bodyWorld * part->mBoneFromBody;
            poses.emplace_back(part->mBone,
                osg::Matrixf::scale(part->mBoneScale) * osg::Matrixf::rotate(Misc::Convert::toOsg(bone.getRotation()))
                    * osg::Matrixf::translate(Misc::Convert::toOsg(bone.getOrigin())));
        }
        mLastPoses = poses;
        return poses;
    }

    void Ragdoll::updatePtr(const MWWorld::Ptr& updated)
    {
        for (const auto& part : mParts)
            part->updatePtr(updated);
    }

    std::shared_ptr<PtrHolder> Ragdoll::findPart(const osg::Vec3f& eye, const osg::Vec3f& direction) const
    {
        std::shared_ptr<PtrHolder> best;
        float bestDistance = std::numeric_limits<float>::max();
        for (const auto& part : mParts)
        {
            btTransform bodyWorld;
            part->mMotionState->getWorldTransform(bodyWorld);
            const osg::Vec3f toCenter = Misc::Convert::toOsg(bodyWorld.getOrigin()) - eye;
            const float along = std::max(toCenter * direction, 0.f);
            const float distance = (toCenter - direction * along).length();
            if (distance < bestDistance)
            {
                bestDistance = distance;
                best = part;
            }
        }
        return best;
    }
}
