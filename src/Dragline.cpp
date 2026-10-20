#include "Dragline.h"

#include <foundation/PxMathUtils.h>

#include <algorithm>
#include <cmath>

using namespace physx;

namespace
{
template <typename T> void safeRelease(T*& p)
{
    if (p) { p->release(); p = nullptr; }
}

// Group/mask filtering (word0 = own group, word1 = groups it collides with), CCD on every
// colliding pair so the thin spreader, walls and chain links cannot tunnel through each other.
PxFilterFlags rigFilterShader(PxFilterObjectAttributes, PxFilterData d0, PxFilterObjectAttributes, PxFilterData d1,
                              PxPairFlags& pairFlags, const void*, PxU32)
{
    if (!(d0.word0 & d1.word1) || !(d1.word0 & d0.word1)) return PxFilterFlag::eSUPPRESS;
    pairFlags = PxPairFlag::eCONTACT_DEFAULT | PxPairFlag::eDETECT_CCD_CONTACT;
    return PxFilterFlag::eDEFAULT;
}

void setFilter(PxRigidActor& actor, PxU32 group, PxU32 collideWith)
{
    const PxFilterData data(group, collideWith, 0, 0);
    PxShape*           shapes[16];
    const PxU32        n = actor.getShapes(shapes, 16);
    for (PxU32 i = 0; i < n; ++i) shapes[i]->setSimulationFilterData(data);
}

void addShape(PxRigidActor& actor, PxMaterial& material, const PxGeometry& geometry, const PxTransform& local)
{
    PxShape* shape = PxRigidActorExt::createExclusiveShape(actor, geometry, material);
    shape->setLocalPose(local);
}

rig::Vec3 toRig(const PxVec3& v) { return rig::Vec3(v.x, v.y, v.z); }
rig::Quat toRig(const PxQuat& q) { return rig::Quat(q.w, q.x, q.y, q.z); }
PxVec3    toPx(const rig::Vec3& v)
{
    return PxVec3(static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()));
}
PxQuat toPx(const rig::Quat& q)
{
    return PxQuat(static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()),
                  static_cast<float>(q.w()));
}

// chain attachment eyes, body-local
PxVec3 yokeEye() { return PxVec3(0.0f, -cfg::kYokeHalf.y, 0.0f); }
PxVec3 barEye(float side) { return PxVec3(side * cfg::kSpreaderHalfW, 0.0f, 0.0f); }
PxVec3 hoistPin(float side) { return PxVec3(side * cfg::kHoistPin.x, cfg::kHoistPin.y, cfg::kHoistPin.z); }
PxVec3 mouthLug(float side) { return PxVec3(side * cfg::kMouthLug.x, cfg::kMouthLug.y, cfg::kMouthLug.z); }
PxVec3 socketChainEye(float side) { return PxVec3(side * cfg::kDragSocketEyeHalfW, 0.0f, 0.0f); }
PxVec3 socketRopeEye() { return PxVec3(0.0f, 0.0f, -cfg::kDragSocketHalf.z); }

// link i of a straight chain of links of length l from a toward b; link-local +X points a -> b
PxTransform straightLinkPose(const PxVec3& a, const PxVec3& b, float l, int i)
{
    PxVec3 dir = b - a;
    dir        = dir.magnitude() > 1e-4f ? dir.getNormalized() : PxVec3(0.0f, -1.0f, 0.0f);
    return PxTransform(a + dir * (l * (static_cast<float>(i) + 0.5f)), PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), dir));
}
} // namespace

HoistDrumParams Dragline::hoistParams()
{
    HoistDrumParams p;
    p.minPayout = cfg::kHoistLenMin;
    p.maxPayout = cfg::kHoistLenMax;
    return p;
}

// Drag drum: stronger than the hoist (it pulls the bucket through the dig), no free fall.
HoistDrumParams Dragline::dragParams()
{
    HoistDrumParams p;
    p.motorForce = 200000.0f;
    p.motorGain  = 80000.0f;
    p.maxSpeed   = 2.5f;
    p.holdBrake  = 200000.0f;
    p.minPayout  = cfg::kDragLenMin;
    p.maxPayout  = cfg::kDragLenMax;
    return p;
}

bool Dragline::init(cfg::ChainModel model)
{
    model_ = model;

    foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, allocator_, errorCallback_);
    if (!foundation_) return false;

    physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation_, PxTolerancesScale(), true);
    if (!physics_) return false;
    extensions_ = PxInitExtensions(*physics_, nullptr);

    PxSceneDesc desc(physics_->getTolerancesScale());
    desc.gravity       = PxVec3(0.0f, -cfg::kGravity, 0.0f);
    dispatcher_        = PxDefaultCpuDispatcherCreate(1);
    desc.cpuDispatcher = dispatcher_;
    desc.filterShader  = rigFilterShader;
    desc.solverType    = PxSolverType::eTGS; // must match the main sim
    desc.flags |= PxSceneFlag::eENABLE_CCD;
    desc.flags |= PxSceneFlag::eENABLE_STABILIZATION; // calms resting contacts (chains lying on bucket/ground)
    scene_ = physics_->createScene(desc);
    if (!scene_) return false;

    // static friction, dynamic friction, restitution
    material_ = physics_->createMaterial(0.6f, 0.5f, 0.05f);

    // ground + lower works (static)
    PxRigidStatic* ground = PxCreatePlane(*physics_, PxPlane(0.0f, 1.0f, 0.0f, 0.0f), *material_);
    setFilter(*ground, cfg::kGroupWorld, cfg::kCollideAll);
    scene_->addActor(*ground);
    for (const float side : {-1.0f, 1.0f})
    {
        const PxVec3   c(side * cfg::kCrawlerOffsetX, cfg::kCrawlerHalf.y, 0.0f);
        PxRigidStatic* crawler = PxCreateStatic(*physics_, PxTransform(c), PxBoxGeometry(cfg::kCrawlerHalf), *material_);
        setFilter(*crawler, cfg::kGroupWorld, cfg::kCollideAll);
        scene_->addActor(*crawler);
    }

    // house: kinematic, driven by the swing model
    house_ = physics_->createRigidDynamic(housePose());
    house_->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
    addShape(*house_, *material_, PxBoxGeometry(cfg::kHouseHalf), PxTransform(cfg::kHouseCenter));
    setFilter(*house_, cfg::kGroupMachine, cfg::kCollideAll);
    scene_->addActor(*house_);

    // hoist rigging bodies and chains
    if (model_ == cfg::ChainModel::Direct)
    {
        createDirectRig(); // the hoist rope is a solver row
        placeRig();
        return true;
    }
    if (model_ == cfg::ChainModel::Articulation) createArticulatedRig();
    else createMaximalRig();

    // hoist rope: boom tip sheave -> top of yoke, pull-only soft limit
    hoistLine_ =PxDistanceJointCreate(*physics_, house_, PxTransform(cfg::boomTipLocal()), yoke_,
                                       PxTransform(PxVec3(0.0f, cfg::kYokeHalf.y, 0.0f)));
    hoistLine_->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
    hoistLine_->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, false);
    hoistLine_->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, true);
    hoistLine_->setDamping(cfg::kRopeDamping);
    hoistLine_->setTolerance(0.0f);
    setHoistLength(drum_.payout());

    placeRig();
    return true;
}

void Dragline::shutdown()
{
    if (artCache_) { art_->releaseCache(*artCache_); artCache_ = nullptr; }
    art_ = nullptr;      // released with the scene
    safeRelease(scene_); // releases actors and joints
    safeRelease(dispatcher_);
    if (extensions_) { PxCloseExtensions(); extensions_ = false; }
    safeRelease(physics_);
    safeRelease(foundation_);
}

void Dragline::setupBody(PxRigidBody& body)
{
    body.setLinearDamping(0.02f);
    body.setAngularDamping(cfg::kRigAngularDamping); // stands in for pin / chain-link friction
    body.setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
    body.setMaxDepenetrationVelocity(cfg::kMaxDepenetrationVel); // links squeezed under the bucket were kicked out at m/s
}

PxRigidDynamic* Dragline::createBody()
{
    PxRigidDynamic* body = physics_->createRigidDynamic(PxTransform(PxIdentity));
    setupBody(*body);
    body->setSolverIterationCounts(cfg::kSolverPosIters, cfg::kSolverVelIters);
    body->setSleepThreshold(0.0f); // rig is driven through joints whose limits change: never sleep
    scene_->addActor(*body);
    return body;
}

void Dragline::addRigBox(PxRigidBody& body, const PxVec3& half, float mass)
{
    addShape(body, *material_, PxBoxGeometry(half), PxTransform(PxIdentity));
    setFilter(body, cfg::kGroupRig, cfg::kCollideAll);
    PxRigidBodyExt::setMassAndUpdateInertia(body, mass);
}

void Dragline::addBucketShell(PxRigidBody& bucket)
{
    for (const cfg::BoxPart& part : cfg::kBucketParts)
        addShape(bucket, *material_, PxBoxGeometry(part.half), PxTransform(part.center));
    setFilter(bucket, cfg::kGroupRig, cfg::kCollideAll);
    PxRigidBodyExt::setMassAndUpdateInertia(bucket, cfg::kBucketMass); // COM from the shell shapes
}

void Dragline::addLinkShape(PxRigidBody& link, float linkLength)
{
    const float r = cfg::kChainLinkRadius;
    addShape(link, *material_, PxCapsuleGeometry(r, std::max(0.5f * linkLength - r, 0.01f)), PxTransform(PxIdentity));
    setFilter(link, cfg::kGroupChain, cfg::kCollideChain);
    PxRigidBodyExt::setMassAndUpdateInertia(link, cfg::kChainMassPerM * linkLength);
    link.setMassSpaceInertiaTensor(link.getMassSpaceInertiaTensor() * cfg::kChainInertiaScale);
}

void Dragline::createChain(ChainRig& chain, PxRigidActor* a, const PxVec3& eyeA, PxRigidActor* b, const PxVec3& eyeB,
                           float length)
{
    chain.a      = a;
    chain.eyeA   = eyeA;
    chain.b      = b;
    chain.eyeB      = eyeB;
    chain.length    = length;
    chain.linkCount = cfg::kChainLinks;

    const float  l = length / static_cast<float>(cfg::kChainLinks);
    const PxVec3 endA(-0.5f * l, 0.0f, 0.0f); // link-local joint points; capsule axis is +X
    const PxVec3 endB(0.5f * l, 0.0f, 0.0f);

    for (int i = 0; i < cfg::kChainLinks; ++i)
    {
        PxRigidDynamic* link = createBody();
        addLinkShape(*link, l);
        chain.links.push_back(link);
    }

    // eye A -> link 0 -> ... -> link N-1 -> eye B (jointed pairs do not collide with each other)
    PxSphericalJointCreate(*physics_, a, PxTransform(eyeA), chain.links.front(), PxTransform(endA));
    for (size_t i = 0; i + 1 < chain.links.size(); ++i)
        PxSphericalJointCreate(*physics_, chain.links[i], PxTransform(endB), chain.links[i + 1], PxTransform(endA));
    PxSphericalJointCreate(*physics_, chain.links.back(), PxTransform(endB), b, PxTransform(eyeB));
}

// Maximal coordinates: every body and link is a PxRigidDynamic, all joints iterative.
void Dragline::createMaximalRig()
{
    yoke_ = createBody();
    addRigBox(*yoke_, cfg::kYokeHalf, cfg::kYokeMass);
    spreader_ = createBody();
    addRigBox(*spreader_, cfg::kSpreaderHalf, cfg::kSpreaderMass);
    bucket_ = createBody();
    addBucketShell(*bucket_);

    // chains: yoke -> spreader ends -> bucket hoist pins
    for (const float side : {1.0f, -1.0f})
    {
        createChain(chains_[side > 0.0f ? UpperL : UpperR], yoke_, yokeEye(), spreader_, barEye(side),
                    cfg::kUpperChainLen);
        createChain(chains_[side > 0.0f ? LowerL : LowerR], spreader_, barEye(side), bucket_, hoistPin(side),
                    cfg::kLowerChainLen);
    }
}

PxArticulationLink* Dragline::addArticulationLink(PxArticulationLink* parent, const PxTransform& parentPose,
                                                  const PxTransform& pose, const PxTransform& childFrame,
                                                  bool twistLimited)
{
    PxArticulationLink* link = art_->createLink(parent, pose);
    setupBody(*link);
    if (!parent) return link;

    // spherical joint whose zero angles are the poses given here (the rig is built hanging straight)
    auto* joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
    joint->setJointType(PxArticulationJointType::eSPHERICAL);
    joint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
    joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
    if (twistLimited)
    {
        joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
        joint->setLimit(PxArticulationAxis::eTWIST, -cfg::kChainTwistLimit, cfg::kChainTwistLimit);
    }
    else
        joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
    joint->setChildPose(childFrame);
    joint->setParentPose(parentPose.getInverse() * pose * childFrame);
    joint->setFrictionCoefficient(cfg::kChainJointFriction);
    return link;
}

// Chain grown from eye B (on the tree parent) back to eye A: link N-1 hangs off the parent,
// every other link off its B-side neighbour. chain.links stays ordered A -> B like the maximal rig.
void Dragline::createChainBranch(ChainRig& chain, PxArticulationLink* parent, const PxTransform& parentPose,
                                 const PxVec3& a, const PxVec3& b, float length)
{
    chain.length    = length;
    chain.linkCount = cfg::kChainLinks;
    const float       l = length / static_cast<float>(cfg::kChainLinks);
    const PxTransform endB(PxVec3(0.5f * l, 0.0f, 0.0f));

    chain.links.assign(cfg::kChainLinks, nullptr);
    PxArticulationLink* prev     = parent;
    PxTransform         prevPose = parentPose;
    for (int i = cfg::kChainLinks - 1; i >= 0; --i)
    {
        const PxTransform   pose = straightLinkPose(a, b, l, i);
        PxArticulationLink* link = addArticulationLink(prev, prevPose, pose, endB, prev != parent);
        addLinkShape(*link, l);
        chain.links[i] = link;
        prev           = link;
        prevPose       = pose;
    }
}

// Reduced coordinates: one floating-base articulation rooted at the bucket. The tree runs
// bucket -> lower L -> spreader -> upper L -> yoke, with upper R (off the spreader) and lower R
// (off the bucket) as open branches closed by ordinary spherical joints.
void Dragline::createArticulatedRig()
{
    art_ = physics_->createArticulationReducedCoordinate();
    art_->setArticulationFlag(PxArticulationFlag::eFIX_BASE, false);
    art_->setSolverIterationCounts(cfg::kArtPosIters, cfg::kArtVelIters);
    art_->setSleepThreshold(0.0f);

    // root first: its centre of mass sets the hanging poses everything else is built in
    PxArticulationLink* bucket = addArticulationLink(nullptr, PxTransform(PxIdentity), PxTransform(PxIdentity),
                                                     PxTransform(PxIdentity), false);
    addBucketShell(*bucket);
    bucket_ = bucket;

    const RigPoses p      = hangingPoses();
    const PxVec3   yokeW  = p.yoke.transform(yokeEye());
    const PxVec3   barLW  = p.spreader.transform(barEye(1.0f));
    const PxVec3   barRW  = p.spreader.transform(barEye(-1.0f));
    const PxVec3   pinLW  = p.bucket.transform(hoistPin(1.0f));
    const PxVec3   pinRW  = p.bucket.transform(hoistPin(-1.0f));
    const float    upperL = cfg::kUpperChainLen / static_cast<float>(cfg::kChainLinks);
    const float    lowerL = cfg::kLowerChainLen / static_cast<float>(cfg::kChainLinks);

    createChainBranch(chains_[LowerL], bucket, p.bucket, barLW, pinLW, cfg::kLowerChainLen);
    PxArticulationLink* spreader = addArticulationLink(static_cast<PxArticulationLink*>(chains_[LowerL].links[0]),
                                                       straightLinkPose(barLW, pinLW, lowerL, 0), p.spreader,
                                                       PxTransform(barEye(1.0f)), false);
    addRigBox(*spreader, cfg::kSpreaderHalf, cfg::kSpreaderMass);
    spreader_ = spreader;

    createChainBranch(chains_[UpperL], spreader, p.spreader, yokeW, barLW, cfg::kUpperChainLen);
    PxArticulationLink* yoke = addArticulationLink(static_cast<PxArticulationLink*>(chains_[UpperL].links[0]),
                                                   straightLinkPose(yokeW, barLW, upperL, 0), p.yoke,
                                                   PxTransform(yokeEye()), false);
    addRigBox(*yoke, cfg::kYokeHalf, cfg::kYokeMass);
    yoke_ = yoke;

    createChainBranch(chains_[UpperR], spreader, p.spreader, yokeW, barRW, cfg::kUpperChainLen);
    createChainBranch(chains_[LowerR], bucket, p.bucket, barRW, pinRW, cfg::kLowerChainLen);

    // loop closures: the only iterative joints left in the rig (besides the rope)
    PxSphericalJointCreate(*physics_, chains_[UpperR].links[0], PxTransform(PxVec3(-0.5f * upperL, 0.0f, 0.0f)), yoke_,
                           PxTransform(yokeEye()));
    PxSphericalJointCreate(*physics_, chains_[LowerR].links[0], PxTransform(PxVec3(-0.5f * lowerL, 0.0f, 0.0f)),
                           spreader_, PxTransform(barEye(-1.0f)));

    for (const float side : {1.0f, -1.0f})
    {
        ChainRig& upper = chains_[side > 0.0f ? UpperL : UpperR];
        upper.a         = yoke_;
        upper.eyeA      = yokeEye();
        upper.b         = spreader_;
        upper.eyeB      = barEye(side);
        ChainRig& lower = chains_[side > 0.0f ? LowerL : LowerR];
        lower.a         = spreader_;
        lower.eyeA      = barEye(side);
        lower.b         = bucket_;
        lower.eyeB      = hoistPin(side);
    }

    scene_->addArticulation(*art_);
    artCache_ = art_->createCache();
}

// The whole rig plumb below the boom tip at the current payout, every chain straight.
Dragline::RigPoses Dragline::hangingPoses() const
{
    const PxQuat yaw = housePose().q; // rig faces the machine: bucket mouth (-Z local) toward the house

    const PxVec3 yokeTop = boomTipWorld() - PxVec3(0.0f, drum_.payout(), 0.0f);
    const PxVec3 yokeC   = yokeTop - PxVec3(0.0f, cfg::kYokeHalf.y, 0.0f);

    const float upperDrop =
        std::sqrt(cfg::kUpperChainLen * cfg::kUpperChainLen - cfg::kSpreaderHalfW * cfg::kSpreaderHalfW);
    const PxVec3 spreaderC = yokeC - PxVec3(0.0f, cfg::kYokeHalf.y + upperDrop, 0.0f);

    // Bucket in its hanging equilibrium: pitched about the pin axis until the centre of mass is
    // plumb below the pins. Spawning it level makes it tip over and rock the whole rig.
    // (The drag side spawns taut and the rope carries most of it, so it is left out here. Hung
    // slack from the mouth lugs instead, its weight would tip the bucket mouth-down into the ground.)
    const PxVec3 pinAxis(0.0f, cfg::kHoistPin.y, cfg::kHoistPin.z);
    const PxVec3 lugAxis(0.0f, cfg::kMouthLug.y, cfg::kMouthLug.z);
    const PxVec3 com   = bucket_->getCMassLocalPose().p - pinAxis;
    const float  pitch = PxPi - std::atan2(com.z, com.y); // rotate (y,z) so com points to -y
    const PxQuat bucketQ = yaw * PxQuat(pitch, PxVec3(1.0f, 0.0f, 0.0f));
    const PxVec3 pinTarget = spreaderC - PxVec3(0.0f, cfg::kLowerChainLen, 0.0f);
    const PxVec3 bucketO   = pinTarget - bucketQ.rotate(pinAxis);

    // socket out from the lug axis toward the fairlead, the drag chains converging on its side eyes
    // and its rope eye (-Z) facing the fairlead
    const PxVec3 lugW     = PxTransform(bucketO, bucketQ).transform(lugAxis);
    const PxVec3 toFair   = (housePose().transform(cfg::kFairlead) - lugW).getNormalized();
    const float  inset    = cfg::kMouthLug.x - cfg::kDragSocketEyeHalfW;
    const float  reach    = std::sqrt(cfg::kDragChainLen * cfg::kDragChainLen - inset * inset);
    const PxVec3 socketC  = lugW + toFair * reach;
    const PxQuat socketQ  = yaw * PxShortestRotation(PxVec3(0.0f, 0.0f, -1.0f), yaw.rotateInv(toFair));

    return {PxTransform(yokeC, yaw), PxTransform(spreaderC, yaw), PxTransform(bucketO, bucketQ),
            PxTransform(socketC, socketQ)};
}

void Dragline::placeRig()
{
    const RigPoses p = hangingPoses();
    if (art_)
    {
        // the tree was built hanging: zero joint angles and velocities, root at the bucket pose
        art_->zeroCache(*artCache_);
        artCache_->rootLinkData->transform = p.bucket;
        art_->applyCache(*artCache_,
                         PxArticulationCache::ePOSITION | PxArticulationCache::eVELOCITY | PxArticulationCache::eROOT);
        return;
    }
    if (model_ == cfg::ChainModel::Direct)
    {
        bucket_->setGlobalPose(p.bucket);
        bucket_->setLinearVelocity(PxVec3(0.0f));
        bucket_->setAngularVelocity(PxVec3(0.0f));
        bucketAccel_    = PxVec3(0.0f, -cfg::kGravity, 0.0f); // hanging: nothing but gravity besides the rig
        bucketAngAccel_ = PxVec3(0.0f);

        auto place = [&](int body, const PxTransform& pose) {
            rig::Body& b = rig_.bodies[body];
            b.x          = toRig(pose.p);
            b.q          = toRig(pose.q);
            b.v = b.w = rig::Vec3::Zero();
        };
        place(rigYoke_, p.yoke);
        place(rigSpreader_, p.spreader);
        if (hasDrag()) place(rigSocket_, p.socket);
        for (ChainRig& chain : chains_)
        {
            if (chain.linkCount == 0) continue;
            PxVec3 a, b;
            chainEnds(static_cast<Chain>(&chain - chains_), a, b);
            const float l = chain.length / static_cast<float>(chain.linkCount);
            for (int i = 0; i < chain.linkCount; ++i) place(chain.firstLink + i, straightLinkPose(a, b, l, i));
        }
        if (hasDrag()) // drag rope at its spawn length: just taut
        {
            PxVec3 fairlead, eye;
            dragEnds(fairlead, eye);
            dragDrum_.reset((eye - fairlead).magnitude() + cfg::kDragInitialSlack);
        }
        rig_.clearWarmStart();
        if (hasDrag()) settleDirect();
        return;
    }

    const PxTransform poses[]  = {p.yoke, p.spreader, p.bucket};
    PxRigidBody*      bodies[] = {yoke_, spreader_, bucket_};
    for (int i = 0; i < 3; ++i)
    {
        bodies[i]->setGlobalPose(poses[i]);
        bodies[i]->setLinearVelocity(PxVec3(0.0f));
        bodies[i]->setAngularVelocity(PxVec3(0.0f));
    }
    for (ChainRig& chain : chains_)
        if (!chain.links.empty()) placeChain(chain);
}

void Dragline::placeChain(ChainRig& chain)
{
    const PxVec3 a = chain.a->getGlobalPose().transform(chain.eyeA);
    const PxVec3 b = chain.b->getGlobalPose().transform(chain.eyeB);
    const float  l = chainLinkLength(static_cast<Chain>(&chain - chains_));
    for (size_t i = 0; i < chain.links.size(); ++i)
    {
        chain.links[i]->setGlobalPose(straightLinkPose(a, b, l, static_cast<int>(i)));
        chain.links[i]->setLinearVelocity(PxVec3(0.0f));
        chain.links[i]->setAngularVelocity(PxVec3(0.0f));
    }
}

// Direct model: a box body (yoke, spreader) in the solver. It hits chain links and other bodies with
// its box, and the ground and other boxes with spheres sampled over its surface.
int Dragline::addRigBoxBody(const PxVec3& half, float mass)
{
    rig::Body b;
    b.invMass = 1.0 / mass;
    const rig::Vec3 h = toRig(half);
    b.invInertia      = rig::Vec3(3.0 / (mass * (h.y() * h.y() + h.z() * h.z())),
                                  3.0 / (mass * (h.x() * h.x() + h.z() * h.z())),
                                  3.0 / (mass * (h.x() * h.x() + h.y() * h.y())));
    b.linDamping      = 0.02;
    b.angDamping      = cfg::kRigAngularDamping;
    b.group           = cfg::kGroupRig;
    b.collideWith     = cfg::kCollideAll;
    b.boxes.push_back({rig::Vec3::Zero(), rig::Quat::Identity(), h});

    const double r = 0.5 * h.minCoeff();
    b.sphereRadius = r;
    const rig::Vec3 inset = (h.array() - r).max(0.0).matrix();
    int             count[3];
    for (int k = 0; k < 3; ++k) count[k] = inset[k] > 0.0 ? static_cast<int>(std::ceil(inset[k] / r)) + 1 : 1;
    for (int i = 0; i < count[0]; ++i)
        for (int j = 0; j < count[1]; ++j)
            for (int k = 0; k < count[2]; ++k)
            {
                const int idx[3] = {i, j, k};
                bool      onSurface = false;
                rig::Vec3 p;
                for (int a = 0; a < 3; ++a)
                {
                    p[a] = count[a] > 1 ? -inset[a] + 2.0 * inset[a] * idx[a] / (count[a] - 1) : 0.0;
                    onSurface |= count[a] > 1 && (idx[a] == 0 || idx[a] == count[a] - 1);
                }
                if (onSurface) b.spheres.push_back(p);
            }
    return rig_.addBody(b);
}

// Direct model: chain links as solver bodies (COM = link centre), ball joints eye A -> links -> eye B.
void Dragline::createDirectChain(ChainRig& chain, int aBody, const PxVec3& eyeA, int bBody, const PxVec3& eyeB,
                                 float length, int links, float massPerM)
{
    chain.aBody     = aBody;
    chain.eyeA      = eyeA;
    chain.bBody     = bBody;
    chain.eyeB      = eyeB;
    chain.length    = length;
    chain.linkCount = links;

    const double l = length / static_cast<double>(links);
    const double r = cfg::kChainLinkRadius;
    const double m = massPerM * l;
    // capsule as a solid cylinder, link inertia boosted like the PhysX links
    const double iAxis = 0.5 * m * r * r * cfg::kChainInertiaScale;
    const double iPerp = m * (3.0 * r * r + l * l) / 12.0 * cfg::kChainInertiaScale;

    rig::Body link;
    link.invMass     = 1.0 / m;
    link.invInertia  = rig::Vec3(1.0 / iAxis, 1.0 / iPerp, 1.0 / iPerp);
    link.linDamping  = 0.02;
    link.angDamping  = cfg::kRigAngularDamping;
    link.group       = cfg::kGroupChain;
    link.collideWith = cfg::kCollideChain;
    link.sphereRadius = r;
    const double seg  = std::max(l - 2.0 * r, 0.0); // sphere centres along the capsule axis, at most r apart
    const int    n    = static_cast<int>(std::ceil(seg / r)) + 1;
    for (int i = 0; i < n; ++i) link.spheres.push_back(rig::Vec3(n > 1 ? -0.5 * seg + seg * i / (n - 1) : 0.0, 0, 0));

    chain.firstLink = static_cast<int>(rig_.bodies.size());
    for (int i = 0; i < links; ++i) rig_.addBody(link);

    // the bucket eye is actor-local; solver anchors are in the body (COM) frame
    auto anchor = [&](int body, const PxVec3& eye) {
        return body == rigBucket_ ? toRig(bucket_->getCMassLocalPose().transformInv(eye)) : toRig(eye);
    };
    const rig::Vec3 endA(-0.5 * l, 0.0, 0.0), endB(0.5 * l, 0.0, 0.0);
    rig_.addBall(aBody, anchor(aBody, eyeA), chain.firstLink, endA);
    for (int i = 0; i + 1 < links; ++i) rig_.addBall(chain.firstLink + i, endB, chain.firstLink + i + 1, endA);
    rig_.addBall(chain.firstLink + links - 1, endB, bBody, anchor(bBody, eyeB));
}

// Direct solve (spec option A): yoke, spreader and chain links live in RigSolver; the bucket stays a
// PhysX body (ground contact) and joins every solve as a proxy with its real mass and inertia.
void Dragline::createDirectRig()
{
    bucket_ = createBody();
    addBucketShell(*bucket_);

    rig_.params.gravity     = rig::Vec3(0.0, -cfg::kGravity, 0.0);
    rig_.params.groundGroup  = cfg::kGroupWorld;
    rig_.params.jointDamping = cfg::kChainJointDamping;

    rig::Body bucket;
    bucket.kind        = rig::BodyKind::Proxy;
    bucket.group       = cfg::kGroupRig;
    bucket.collideWith = cfg::kCollideAll;
    const PxTransform comInv = bucket_->getCMassLocalPose().getInverse();
    for (const cfg::BoxPart& part : cfg::kBucketParts)
    {
        const PxTransform local = comInv * PxTransform(part.center);
        bucket.boxes.push_back({toRig(local.p), toRig(local.q), toRig(part.half)});
    }
    rigBucket_ = rig_.addBody(bucket);

    rig::Body house; // origin on the swing axis: carries the boom tip end of the hoist rope
    house.kind = rig::BodyKind::Kinematic;
    rigHouse_  = rig_.addBody(house);

    rigYoke_     = addRigBoxBody(cfg::kYokeHalf, cfg::kYokeMass);
    rigSpreader_ = addRigBoxBody(cfg::kSpreaderHalf, cfg::kSpreaderMass);
    for (const float side : {1.0f, -1.0f})
    {
        createDirectChain(chains_[side > 0.0f ? UpperL : UpperR], rigYoke_, yokeEye(), rigSpreader_, barEye(side),
                          cfg::kUpperChainLen, cfg::kChainLinks, cfg::kChainMassPerM);
        createDirectChain(chains_[side > 0.0f ? LowerL : LowerR], rigSpreader_, barEye(side), rigBucket_,
                          hoistPin(side), cfg::kLowerChainLen, cfg::kChainLinks, cfg::kChainMassPerM);
    }
    hoistRope_ = rig_.addRope(rigYoke_, rig::Vec3(0.0, cfg::kYokeHalf.y, 0.0), rigHouse_, toRig(cfg::boomTipLocal()));

    // drag side: mouth lugs -> drag chains -> socket -> drag rope -> fairlead on the house
    rigSocket_ = addRigBoxBody(cfg::kDragSocketHalf, cfg::kDragSocketMass);
    for (const float side : {1.0f, -1.0f})
        createDirectChain(chains_[side > 0.0f ? DragL : DragR], rigBucket_, mouthLug(side), rigSocket_,
                          socketChainEye(side), cfg::kDragChainLen, cfg::kDragChainLinks, cfg::kDragChainMassPerM);
    dragRope_ = rig_.addRope(rigSocket_, toRig(socketRopeEye()), rigHouse_, toRig(cfg::kFairlead));
}

void Dragline::solveDirect(float dt)
{
    // bucket proxy: PhysX state + free velocity from last substep's non-rig acceleration (lagged force mode)
    const PxTransform com   = bucket_->getGlobalPose() * bucket_->getCMassLocalPose();
    rig::Body&        proxy = rig_.bodies[rigBucket_];
    bucketV0_               = bucket_->getLinearVelocity();
    bucketW0_               = bucket_->getAngularVelocity();
    proxy.x                 = toRig(com.p);
    proxy.q                 = toRig(com.q);
    proxy.v                 = toRig(bucketV0_ + bucketAccel_ * dt);
    proxy.w                 = toRig(bucketW0_ + bucketAngAccel_ * dt);
    proxy.invMass           = bucket_->getInvMass();
    proxy.invInertia        = toRig(bucket_->getMassSpaceInvInertiaTensor());

    rig::Body& house = rig_.bodies[rigHouse_];
    house.q          = toRig(house_->getGlobalPose().q); // start of the substep; the kinematic target is its end
    house.w          = rig::Vec3(0.0, swingRate_, 0.0);

    lastH_ = dt;
    rig_.setRopeState(hoistRope_, drum_.payout(), drum_.speed(), cfg::kRopeEA / std::max(drum_.payout(), 1.0f),
                      cfg::kRopeDamping);
    rig_.setRopeState(dragRope_, dragDrum_.payout(), dragDrum_.speed(),
                      cfg::kRopeEA / std::max(dragDrum_.payout(), 1.0f), cfg::kRopeDamping);
    if (!rig_.step(dt)) return;

    rig::Vec3 p, l;
    rig_.bodyImpulse(rigBucket_, p, l);
    bucketDv_ = toPx(p) * bucket_->getInvMass();
    bucketDw_ = toPx(proxy.invInertiaWorld() * l);
    bucket_->addForce(toPx(p), PxForceMode::eIMPULSE);
    bucket_->addTorque(toPx(l), PxForceMode::eIMPULSE);
}

void Dragline::afterDirect(float dt)
{
    bucketAccel_    = (bucket_->getLinearVelocity() - bucketV0_ - bucketDv_) / dt;
    bucketAngAccel_ = (bucket_->getAngularVelocity() - bucketW0_ - bucketDw_) / dt;
}

PxTransform Dragline::rigPose(int body) const
{
    const rig::Body& b = rig_.bodies[body];
    return PxTransform(toPx(b.x), toPx(b.q));
}

PxVec3 Dragline::eyeWorld(int body, const PxRigidActor* actor, const PxVec3& eye) const
{
    if (model_ != cfg::ChainModel::Direct) return actor->getGlobalPose().transform(eye);
    return body == rigBucket_ ? bucket_->getGlobalPose().transform(eye) : rigPose(body).transform(eye);
}

void Dragline::reset()
{
    swingAngle_ = 0.0f;
    swingRate_  = 0.0f;
    house_->setGlobalPose(housePose());

    drum_.reset(cfg::kHoistLenInitial);
    setHoistLength(drum_.payout());
    placeRig();
}

void Dragline::step(const DraglineControls& c, float dt)
{
    const float h = dt / static_cast<float>(cfg::kSubSteps);
    for (int i = 0; i < cfg::kSubSteps; ++i) subStep(c, h);
}

// The placed pose is no equilibrium once the drag side is in: a straight rope cannot carry the
// socket without sag, and the drag pull swings the rig toward the machine. Simulate it to rest
// with near-critical damping on every body (the rig pendulum is ~0.9 rad/s), then stop it dead.
void Dragline::settleDirect()
{
    for (rig::Body& b : rig_.bodies)
    {
        b.linDamping += cfg::kSettleLinDamping;
        b.angDamping += cfg::kSettleAngDamping;
    }
    bucket_->setLinearDamping(cfg::kSettleLinDamping);
    bucket_->setAngularDamping(cfg::kSettleAngDamping);

    const DraglineControls hold; // drums on their holding brakes
    const float            h = cfg::kSettleDt / static_cast<float>(cfg::kSubSteps);
    for (float t = 0.0f; t < cfg::kSettleTime; t += cfg::kSettleDt)
        for (int i = 0; i < cfg::kSubSteps; ++i) subStep(hold, h);

    for (rig::Body& b : rig_.bodies)
    {
        b.linDamping -= cfg::kSettleLinDamping;
        b.angDamping -= cfg::kSettleAngDamping;
        b.v = b.w = rig::Vec3::Zero();
    }
    setupBody(*bucket_);
    bucket_->setLinearVelocity(PxVec3(0.0f));
    bucket_->setAngularVelocity(PxVec3(0.0f));
}

void Dragline::subStep(const DraglineControls& c, float dt)
{
    // swing drive: rate ramps toward the commanded rate, limited by swing acceleration
    const float targetRate = std::clamp(c.swing, -1.0f, 1.0f) * cfg::kSwingMaxRate;
    const float maxDelta   = cfg::kSwingAccel * dt;
    swingRate_ += std::clamp(targetRate - swingRate_, -maxDelta, maxDelta);
    swingAngle_ += swingRate_ * dt;

    // hoist rope: the drum carries the rig while the rope is taut, minus the bucket once it is
    // grounded. (Deliberately not keyed on the individual chains: they go slack and taut as
    // the rig jostles, and a flickering drum load feeds the jostling.)
    float loadMass = 0.0f;
    if (ropeTaut())
    {
        loadMass = rigMass();
        if (bucketClearance() <= cfg::kGroundedHeight) loadMass -= cfg::kBucketMass;
    }
    PxVec3 tip, yokeTop;
    hoistEnds(tip, yokeTop);
    const PxVec3 rope = yokeTop - tip;
    const float  dist = rope.magnitude();
    const float  loadForce = dist > 1e-3f ? loadMass * cfg::kGravity * (-rope.y / dist) : 0.0f; // gravity along rope

    drum_.step(c.hoist, loadForce, loadMass, dt);
    setHoistLength(drum_.payout());

    // drag rope: mostly horizontal, so no weight estimate: the drum feels last substep's measured tension
    HoistDrumInput drag = c.drag;
    drag.freeFall       = false;
    dragDrum_.step(drag, dragTension(), 0.0f, dt);

    if (model_ == cfg::ChainModel::Direct) solveDirect(dt);
    house_->setKinematicTarget(housePose());
    scene_->simulate(dt);
    scene_->fetchResults(true);
    if (model_ == cfg::ChainModel::Direct) afterDirect(dt);
}

PxTransform Dragline::housePose() const
{
    return PxTransform(PxVec3(0.0f), PxQuat(swingAngle_, PxVec3(0.0f, 1.0f, 0.0f)));
}

PxTransform Dragline::yokePose() const
{
    return model_ == cfg::ChainModel::Direct ? rigPose(rigYoke_) : yoke_->getGlobalPose();
}

PxTransform Dragline::spreaderPose() const
{
    return model_ == cfg::ChainModel::Direct ? rigPose(rigSpreader_) : spreader_->getGlobalPose();
}

PxTransform Dragline::chainLinkPose(Chain c, int i) const
{
    return model_ == cfg::ChainModel::Direct ? rigPose(chains_[c].firstLink + i) : chains_[c].links[i]->getGlobalPose();
}

void Dragline::hoistEnds(PxVec3& tip, PxVec3& yokeTop) const
{
    tip     = boomTipWorld();
    yokeTop = yokePose().transform(PxVec3(0.0f, cfg::kYokeHalf.y, 0.0f));
}

void Dragline::chainEnds(Chain c, PxVec3& a, PxVec3& b) const
{
    a = eyeWorld(chains_[c].aBody, chains_[c].a, chains_[c].eyeA);
    b = eyeWorld(chains_[c].bBody, chains_[c].b, chains_[c].eyeB);
}

bool Dragline::ropeTaut() const
{
    PxVec3 tip, yokeTop;
    hoistEnds(tip, yokeTop);
    return (yokeTop - tip).magnitude() >= drum_.payout() - cfg::kTautBand;
}

float Dragline::chainLinkLength(Chain c) const
{
    return chains_[c].linkCount > 0 ? chains_[c].length / static_cast<float>(chains_[c].linkCount) : 0.0f;
}

bool Dragline::chainTaut(Chain c) const
{
    if (chains_[c].linkCount == 0) return false;
    // a hanging link chain is never perfectly straight: allow a little sag before calling it slack
    PxVec3 a, b;
    chainEnds(c, a, b);
    return (b - a).magnitude() >= 0.97f * chains_[c].length;
}

float Dragline::rigMass() const
{
    const float chainMass = 2.0f * (cfg::kUpperChainLen + cfg::kLowerChainLen) * cfg::kChainMassPerM;
    return cfg::kYokeMass + cfg::kSpreaderMass + cfg::kBucketMass + chainMass + dragMass();
}

float Dragline::dragMass() const
{
    if (model_ != cfg::ChainModel::Direct) return 0.0f;
    return cfg::kDragSocketMass + 2.0f * cfg::kDragChainLen * cfg::kDragChainMassPerM;
}

PxTransform Dragline::socketPose() const { return hasDrag() ? rigPose(rigSocket_) : PxTransform(PxIdentity); }

void Dragline::dragEnds(PxVec3& fairlead, PxVec3& socketEye) const
{
    fairlead  = housePose().transform(cfg::kFairlead);
    socketEye = hasDrag() ? socketPose().transform(socketRopeEye()) : fairlead;
}

bool Dragline::dragRopeTaut() const
{
    if (!hasDrag()) return false;
    PxVec3 fairlead, eye;
    dragEnds(fairlead, eye);
    return (eye - fairlead).magnitude() >= dragDrum_.payout() - cfg::kTautBand;
}

float Dragline::dragTension() const
{
    return hasDrag() ? static_cast<float>(-rig_.ropeImpulse(dragRope_) / lastH_) : 0.0f;
}

float Dragline::lineTension() const
{
    if (model_ == cfg::ChainModel::Direct) return static_cast<float>(-rig_.ropeImpulse(hoistRope_) / lastH_);
    // spring part of the soft limit (constraint getForce is stale for soft/inactive limits)
    const float stretch = hoistLine_->getRelativeTransform().p.magnitude() - hoistLine_->getMaxDistance();
    return stretch > 0.0f ? hoistLine_->getStiffness() * stretch : 0.0f;
}

float Dragline::bucketClearance() const
{
    // lowest corner of the whole shell: the bucket can rest on its nose, walls or arch, not just the floor
    const PxTransform pose   = bucket_->getGlobalPose();
    float             lowest = PX_MAX_F32;
    for (const cfg::BoxPart& part : cfg::kBucketParts)
        for (const float x : {-1.0f, 1.0f})
            for (const float y : {-1.0f, 1.0f})
                for (const float z : {-1.0f, 1.0f})
                {
                    const PxVec3 corner = part.center + PxVec3(x * part.half.x, y * part.half.y, z * part.half.z);
                    lowest              = std::min(lowest, pose.transform(corner).y);
                }
    return lowest;
}

void Dragline::setHoistLength(float length)
{
    if (!hoistLine_) return; // direct model: the rope is a solver row, updated every substep
    hoistLine_->setMaxDistance(length);
    hoistLine_->setStiffness(cfg::kRopeEA / std::max(length, 1.0f));
}
