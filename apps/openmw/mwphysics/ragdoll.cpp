#include "ragdoll.hpp"

#include <algorithm>
#include <limits>
#include <mutex>
#include <string_view>

#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
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
        }

    private:
        mutable std::mutex mMutex;
        btTransform mTransform;
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

            btVector3 inertia(0, 0, 0);
            part->mShape->calculateLocalInertia(def.mMass, inertia);
            part->mMotionState = std::make_unique<RagdollMotionState>(bodyWorld);
            btRigidBody::btRigidBodyConstructionInfo info(
                def.mMass, part->mMotionState.get(), part->mShape.get(), inertia);
            info.m_friction = 0.8f;
            info.m_restitution = 0.05f;
            info.m_linearDamping = 0.05f;
            info.m_angularDamping = 0.5f;
            // In game units (~70 per meter), like the other simulated objects.
            info.m_linearSleepingThreshold = 4.f;
            info.m_angularSleepingThreshold = 1.f;
            auto body = std::make_unique<btRigidBody>(info);

            body->setCcdMotionThreshold(def.mRadius * 0.5f);
            body->setCcdSweptSphereRadius(def.mRadius * 0.8f);
            body->setUserPointer(part.get());

            // Falling as it was moving, the upper body thrown back by the killing blow.
            btVector3 partVelocity = Misc::Convert::toBullet(velocity);
            if (def.mBone == "Bip01 Spine1" || def.mBone == "Bip01 Head")
                partVelocity += Misc::Convert::toBullet(kick);
            else if (def.mBone == "Bip01 Pelvis")
                partVelocity += Misc::Convert::toBullet(kick) * 0.6f;
            body->setLinearVelocity(partVelocity);

            part->setBody(std::move(body));
            mParts.push_back(std::move(part));
        }

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

        // Joints where each part meets its parent, twisting around the part's own bone.
        for (size_t i = 0; i < mParts.size(); ++i)
        {
            const PartDef& def = sPartDefs[i];
            if (def.mParent < 0)
                continue;
            btRigidBody& parent = *mParts[def.mParent]->mBody;
            btRigidBody& child = *mParts[i]->mBody;
            const btTransform& childWorld = child.getWorldTransform();
            const btVector3 childAxis = childWorld.getBasis() * btVector3(0, 1, 0);
            const btVector3 jointPosition = Misc::Convert::toBullet(position(def.mBone));
            const btTransform jointWorld(shortestArcQuat(btVector3(1, 0, 0), childAxis), jointPosition);
            auto joint = std::make_unique<btConeTwistConstraint>(parent, child,
                parent.getWorldTransform().inverse() * jointWorld, childWorld.inverse() * jointWorld);
            joint->setLimit(btRadians(def.mSwing1), btRadians(def.mSwing2), btRadians(def.mTwist), 0.9f, 0.3f, 1.f);
            mTaskScheduler->addConstraint(joint.get());
            mJoints.push_back(std::move(joint));
        }
    }

    Ragdoll::~Ragdoll()
    {
        for (const auto& joint : mJoints)
            mTaskScheduler->removeConstraint(joint.get());
        for (const auto& part : mParts)
            mTaskScheduler->removeCollisionObject(part->mBody);
    }

    std::vector<std::pair<std::string, osg::Matrixf>> Ragdoll::getBonePoses() const
    {
        std::vector<std::pair<std::string, osg::Matrixf>> poses;
        poses.reserve(mParts.size());
        for (const auto& part : mParts)
        {
            btTransform bodyWorld;
            part->mMotionState->getWorldTransform(bodyWorld);
            const btTransform bone = bodyWorld * part->mBoneFromBody;
            poses.emplace_back(part->mBone,
                osg::Matrixf::scale(part->mBoneScale) * osg::Matrixf::rotate(Misc::Convert::toOsg(bone.getRotation()))
                    * osg::Matrixf::translate(Misc::Convert::toOsg(bone.getOrigin())));
        }
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
