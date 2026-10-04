#ifndef OPENMW_MWPHYSICS_RAGDOLL_H
#define OPENMW_MWPHYSICS_RAGDOLL_H

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <osg/Matrixf>
#include <osg/Vec3f>

#include <LinearMath/btTransform.h>

#include "ptrholder.hpp"

class btCollisionShape;
class btRigidBody;
class btTypedConstraint;

namespace MWPhysics
{
    class PhysicsTaskScheduler;
    class RagdollMotionState;

    /// A dead actor's body as linked simulated parts (pelvis, chest, head, arms, legs), each driving a bone of
    /// the standard (Bip01) skeleton.
    class Ragdoll
    {
    public:
        using BoneMatrices = std::map<std::string, osg::Matrixf, std::less<>>;

        /// The bones this needs, in hierarchy order (parents first). Their world matrices are the starting pose.
        static const std::vector<std::string>& getRequiredBones();

        /// @param bones world matrices of getRequiredBones()
        /// @param velocity of the actor as it died
        /// @param kick velocity change of the upper body from the blow that killed it
        Ragdoll(const MWWorld::Ptr& actor, const BoneMatrices& bones, const osg::Vec3f& velocity,
            const osg::Vec3f& kick, PhysicsTaskScheduler* scheduler);
        ~Ragdoll();

        /// Where the simulated bones are now (world matrices), parents first.
        std::vector<std::pair<std::string, osg::Matrixf>> getBonePoses() const;

        void updatePtr(const MWWorld::Ptr& updated);

        /// The part closest to a view ray from eye along direction (normalized), for grabbing.
        std::shared_ptr<PtrHolder> findPart(const osg::Vec3f& eye, const osg::Vec3f& direction) const;

    private:
        struct Part;

        PhysicsTaskScheduler* mTaskScheduler;
        std::vector<std::shared_ptr<Part>> mParts; // shared: a part can be held (see PhysicsTaskScheduler)
        std::vector<std::unique_ptr<btTypedConstraint>> mJoints;
    };
}

#endif
