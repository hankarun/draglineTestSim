#pragma once

#include "DraglineConfig.h"
#include "HoistDrum.h"
#include "RigSolver.h"

#include <PxPhysicsAPI.h>

#include <vector>

struct DraglineControls
{
    float          swing = 0.0f; // -1 = right .. +1 = left
    HoistDrumInput hoist;
    HoistDrumInput drag; // lever only: -1 = pull in .. +1 = pay out
};

// PhysX scene + dragline machine: kinematic house swinging on a static
// carbody, fixed boom, and the hoist rigging hanging from the boom tip:
// hoist rope -> yoke -> 2 upper chains -> spreader bar -> 2 lower chains -> bucket.
// The rope is a pull-only soft distance joint; each chain is a string of heavy
// capsule links joined by spherical joints, colliding with the rig and the ground.
// Direct model only: the drag side, fairlead -> drag rope -> socket -> 2 drag chains -> mouth lugs.
class Dragline
{
public:
    enum Chain { UpperL, UpperR, LowerL, LowerR, DragL, DragR, ChainCount }; // drag chains: direct model only

    bool init(cfg::ChainModel model = cfg::kDefaultChainModel);
    cfg::ChainModel chainModel() const { return model_; }
    void shutdown();
    void reset();
    void step(const DraglineControls& controls, float dt);

    physx::PxTransform housePose() const;
    physx::PxTransform yokePose() const;
    physx::PxTransform spreaderPose() const;
    physx::PxTransform bucketPose() const { return bucket_->getGlobalPose(); }
    physx::PxVec3      boomTipWorld() const { return housePose().transform(cfg::boomTipLocal()); }

    // world end points of the hoist rope
    void hoistEnds(physx::PxVec3& tip, physx::PxVec3& yokeTop) const;

    // chains: end points (attachment eyes), rated length, and the individual links
    void  chainEnds(Chain c, physx::PxVec3& a, physx::PxVec3& b) const;
    float chainLength(Chain c) const { return chains_[c].length; }
    int   chainLinkCount(Chain c) const { return chains_[c].linkCount; } // 0: chain not built
    float chainLinkLength(Chain c) const;
    physx::PxTransform chainLinkPose(Chain c, int i) const;

    float swingAngle() const { return swingAngle_; }
    float swingRate() const { return swingRate_; }
    bool  ropeTaut() const;
    bool  chainTaut(Chain c) const;
    float lineTension() const; // N
    float bucketClearance() const; // m, lowest point of the bucket shell above the ground
    const HoistDrum& drum() const { return drum_; }

    // drag side (hasDrag() false: not built, the rest returns defaults)
    bool               hasDrag() const { return rigSocket_ >= 0; }
    physx::PxTransform socketPose() const;
    void               dragEnds(physx::PxVec3& fairlead, physx::PxVec3& socketEye) const;
    bool               dragRopeTaut() const;
    float              dragTension() const; // N
    const HoistDrum&   dragDrum() const { return dragDrum_; }
    void               setDragMotorForce(float force) { dragDrum_.setMotorForce(force); } // N, max drag line pull

    // live tuning (app sliders)
    void  setHoistMotorForce(float force) { drum_.setMotorForce(force); } // N, max hoist line pull
    void  setSwingMaxRate(float rate) { swingMaxRate_ = rate; }          // rad/s at full swing lever
    float swingMaxRate() const { return swingMaxRate_; }

    // dump rope (with the drag side): bucket arch -> dump block sheave (block hangs from the yoke) -> socket top
    physx::PxTransform dumpBlockPose() const { return hasDrag() ? rigPose(rigDumpBlock_) : physx::PxTransform(physx::PxIdentity); }
    void  dumpEnds(physx::PxVec3& arch, physx::PxVec3& sheave, physx::PxVec3& socketEye) const;
    float dumpRopeLength() const { return dumpLength_; } // fixed, sized at spawn for cfg::kCarryPitchDeg
    bool  dumpRopeTaut() const;
    float dumpTension() const; // N
    float bucketPitch() const; // rad, mouth direction above the horizontal (> 0 = mouth up)

private:
    // one chain: links run from eye A (on actor a) to eye B (on actor b); link-local +X points A -> B.
    // Direct model: no PhysX links; the ends are solver bodies (aBody/bBody, eyes in their frames,
    // except the bucket, whose eye stays actor-local) and the links follow firstLink.
    struct ChainRig
    {
        std::vector<physx::PxRigidBody*> links;
        physx::PxRigidActor* a = nullptr;
        physx::PxVec3        eyeA{0.0f};
        physx::PxRigidActor* b = nullptr;
        physx::PxVec3        eyeB{0.0f};
        float                length = 0.0f;
        int                  linkCount = 0;
        int                  aBody = -1, bBody = -1, firstLink = -1;
    };

    struct RigPoses
    {
        physx::PxTransform yoke, spreader, bucket, socket, dumpBlock; // socket, dump block: direct model only
    };

    void setupBody(physx::PxRigidBody& body);  // damping, CCD, depenetration cap
    physx::PxRigidDynamic* createBody(); // dynamic rig body, CCD on, never sleeps
    void addRigBox(physx::PxRigidBody& body, const physx::PxVec3& half, float mass);
    void addBucketShell(physx::PxRigidBody& bucket);
    void addLinkShape(physx::PxRigidBody& link, float linkLength);
    void createChain(ChainRig& chain, physx::PxRigidActor* a, const physx::PxVec3& eyeA, physx::PxRigidActor* b,
                     const physx::PxVec3& eyeB, float length);
    void createMaximalRig();

    // articulation: link pinned to `parent` at childFrame (link-local); zero joint angles = current poses
    physx::PxArticulationLink* addArticulationLink(physx::PxArticulationLink* parent, const physx::PxTransform& parentPose,
                                                   const physx::PxTransform& pose, const physx::PxTransform& childFrame,
                                                   bool twistLimited);
    void createChainBranch(ChainRig& chain, physx::PxArticulationLink* parent, const physx::PxTransform& parentPose,
                           const physx::PxVec3& a, const physx::PxVec3& b, float length);
    void createArticulatedRig();

    // direct model (own solver, RigSolver.h)
    int  addRigBoxBody(const physx::PxVec3& half, float mass);
    void createDirectChain(ChainRig& chain, int aBody, const physx::PxVec3& eyeA, int bBody, const physx::PxVec3& eyeB,
                           float length, int links, float massPerM);
    void createDirectRig();
    void solveDirect(float dt); // before the PhysX step: proxy in, solve, impulse on the bucket out
    void afterDirect(float dt); // after it: the bucket's non-rig acceleration for the next prediction
    physx::PxTransform rigPose(int body) const;
    physx::PxVec3      eyeWorld(int body, const physx::PxRigidActor* actor, const physx::PxVec3& eye) const;

    RigPoses hangingPoses() const;
    void placeRig();
    void placeChain(ChainRig& chain);
    void setHoistLength(float length); // rope length + its length-dependent stiffness
    void subStep(const DraglineControls& controls, float dt);
    void settleDirect(); // direct model with a drag side: run the placed rig to rest
    float rigMass() const;
    float dragMass() const; // drag chains + socket, 0 without a drag side
    static HoistDrumParams hoistParams();
    static HoistDrumParams dragParams();

    physx::PxDefaultAllocator     allocator_;
    physx::PxDefaultErrorCallback errorCallback_;

    physx::PxFoundation*           foundation_ = nullptr;
    physx::PxPhysics*              physics_    = nullptr;
    physx::PxDefaultCpuDispatcher* dispatcher_ = nullptr;
    physx::PxScene*                scene_      = nullptr;
    physx::PxMaterial*             material_   = nullptr;
    bool                           extensions_ = false;

    physx::PxRigidDynamic*  house_     = nullptr;
    cfg::ChainModel model_ = cfg::kDefaultChainModel;

    physx::PxRigidBody*     yoke_     = nullptr;
    physx::PxRigidBody*     spreader_  = nullptr;
    physx::PxRigidBody*     bucket_    = nullptr;
    physx::PxArticulationReducedCoordinate* art_      = nullptr; // articulation model only
    physx::PxArticulationCache*             artCache_ = nullptr;
    physx::PxDistanceJoint* hoistLine_ = nullptr;
    ChainRig                chains_[ChainCount];

    rig::Solver   rig_; // direct model only
    int           rigBucket_ = -1, rigHouse_ = -1, rigYoke_ = -1, rigSpreader_ = -1, rigSocket_ = -1, rigDumpBlock_ = -1;
    int           hoistRope_ = -1, dragRope_ = -1, dumpRope_ = -1; // solver rope indices
    float         dumpLength_ = 0.0f;
    physx::PxVec3 bucketAccel_{0.0f};    // last substep's bucket acceleration from everything but the rig
    physx::PxVec3 bucketAngAccel_{0.0f};
    physx::PxVec3 bucketV0_{0.0f}, bucketW0_{0.0f}, bucketDv_{0.0f}, bucketDw_{0.0f}; // around the PhysX step
    float         lastH_ = 1.0f;

    HoistDrum drum_{hoistParams(), cfg::kHoistLenInitial};
    HoistDrum dragDrum_{dragParams(), cfg::kDragLenMax}; // payout set when the rig is placed
    float     swingAngle_ = 0.0f;
    float     swingRate_  = 0.0f;
    float     swingMaxRate_ = cfg::kSwingMaxRate;
};
