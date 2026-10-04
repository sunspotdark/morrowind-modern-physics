#ifndef OPENMW_MWPHYSICS_RAGDOLL_H
#define OPENMW_MWPHYSICS_RAGDOLL_H

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <osg/Matrixf>
#include <osg/Vec2f>
#include <osg/Vec3f>

#include <LinearMath/btTransform.h>

#include "ptrholder.hpp"

class btCollisionShape;
class btRigidBody;
class btTypedConstraint;
class btActionInterface;

namespace MWPhysics
{
    class PhysicsTaskScheduler;
    class RagdollMotionState;

    /// A dead actor's body as linked simulated parts (pelvis, chest, head, arms, legs...), each driving a bone of
    /// its skeleton.
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
        /// A body without the standard skeleton, as one box: bodyWorld/halfExtents place it, baseWorld is where the
        /// actor's model is, which then follows it (its pose is the only one, unnamed).
        Ragdoll(const MWWorld::Ptr& actor, const btTransform& bodyWorld, const osg::Vec3f& halfExtents, float mass,
            const osg::Matrixf& baseWorld, PhysicsTaskScheduler* scheduler);
        /// A bone of a skinned mesh, and the bounds (in its own space) of the part of the mesh it moves.
        struct SkinnedBone
        {
            std::string mName;
            int mParent; // index of the closest skinned ancestor, -1 for none
            osg::Matrixf mWorld;
            osg::Vec3f mCenter;
            float mRadius;
        };

        /// A body of any (skinned) shape, its parts fitted to the mesh. Bones are parents first.
        /// @return null if it would hardly have any parts
        static std::unique_ptr<Ragdoll> fromSkeleton(const MWWorld::Ptr& actor, const std::vector<SkinnedBone>& bones,
            const osg::Vec3f& velocity, const osg::Vec3f& kick, PhysicsTaskScheduler* scheduler);

        /// A node of a model built of rigid pieces, and the bounds (in its own space) of the pieces it carries.
        struct RigidPiece
        {
            std::string mName;
            int mParent; // index of the closest such ancestor, -1 for none
            osg::Matrixf mWorld;
            osg::Vec3f mMin;
            osg::Vec3f mMax;
        };

        /// A body built of rigid pieces falls apart: each (big enough) piece is a loose object of its own; the
        /// smaller ones stay on the piece above them. Pieces are parents first.
        /// @return null if it would hardly fall apart
        static std::unique_ptr<Ragdoll> fromRigidPieces(const MWWorld::Ptr& actor,
            const std::vector<RigidPiece>& pieces, const osg::Vec3f& velocity, const osg::Vec3f& kick,
            PhysicsTaskScheduler* scheduler);

        ~Ragdoll();

        /// Whether the parts are joined together (otherwise they're loose pieces).
        bool isJointed() const { return !mJoints.empty(); }

        /// Where the simulated bones are now (world matrices), parents first. Interpolation is how far the time
        /// since the last physics step is to the next one, for smooth movement at any frame rate.
        std::vector<std::pair<std::string, osg::Matrixf>> getBonePoses(float interpolation) const;

        void updatePtr(const MWWorld::Ptr& updated);

        float getMass() const;

        /// Still and asleep, as if it had been lying there (until something disturbs it).
        void setAtRest();

        /// The part closest to a view ray from eye along direction (normalized), for grabbing.
        std::shared_ptr<PtrHolder> findPart(const osg::Vec3f& eye, const osg::Vec3f& direction) const;

        /// For a single body: the point on it nearest to eye, where it is grabbed. Otherwise none.
        std::optional<osg::Vec3f> getGrabPoint(const osg::Vec3f& eye) const;

        /// Whether it has come apart (a model the parts don't suit: parts flying off, joints torn), checked over
        /// its first seconds; duration is the time since the last check.
        bool hasComeApart(float duration);

        MWWorld::Ptr getPtr() const;

        /// What getBonePoses gave last (so the rest of the frame agrees with what the bones were set to).
        const std::vector<std::pair<std::string, osg::Matrixf>>& getLastBonePoses() const { return mLastPoses; }

        /// Where the actor (whose position is at its feet, actorPosition now) should be for its body to be where
        /// the ragdoll is: following the main part (the pelvis, say) along the ground, at its height. Empty
        /// before the bones were first posed.
        std::optional<osg::Vec3f> getActorPosition(const osg::Vec3f& actorPosition);

    private:
        struct Part;

        explicit Ragdoll(PhysicsTaskScheduler* scheduler);
        static void makeBody(Part& part, const btTransform& bodyWorld, float mass, float thickness);
        void addToWorld();
        void addJointFriction(); // once all joints are made
        void addJoint(size_t parentIndex, size_t childIndex, const osg::Vec3f& position, float swing1, float swing2,
            float twist);

        mutable std::vector<std::pair<std::string, osg::Matrixf>> mLastPoses;
        std::unique_ptr<btActionInterface> mJointFriction;
        std::optional<osg::Vec2f> mActorOffset; // from the main part to the actor, along the ground
        // For hasComeApart: how long it has been checked, how far each part started from the main one, and where
        // each part was at the last check.
        float mAge = 0;
        float mCheckTimer = 0;
        std::vector<float> mStartDistances;
        std::vector<btVector3> mLastPositions;

        PhysicsTaskScheduler* mTaskScheduler;
        std::vector<std::shared_ptr<Part>> mParts; // shared: a part can be held (see PhysicsTaskScheduler)
        std::vector<std::unique_ptr<btTypedConstraint>> mJoints;
    };
}

#endif
