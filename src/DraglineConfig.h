#pragma once

#include <PxPhysicsAPI.h>

#include <cmath>

// Machine geometry and tuning shared by the simulation and the renderer.
// Machine frame (= house-local frame): origin on the ground on the swing axis,
// +Z forward (along the boom), +Y up, +X to the operator's left.
// Values marked [xml] come from sampleConfig/LiebherrHS825Phy.xml and
// sampleConfig/LiebherrHS825Dragline.entity (attribute name given).
namespace cfg
{
using physx::PxVec3;

constexpr float kGravity  = 9.81f;
constexpr int   kSubSteps = 4; // physics substeps per frame step: the rig is a stiff chain of light/heavy bodies
constexpr physx::PxU32 kSolverPosIters = 8; // per substep; TGS needs far fewer than PGS
constexpr physx::PxU32 kSolverVelIters = 1;
constexpr float kRigAngularDamping = 1.5f;  // rig bodies: friction in pins and chain links (real rigs stop swinging)
constexpr float kMaxDepenetrationVel = 1.0f; // m/s; PhysX default is effectively unlimited
constexpr float kDegToRad = 0.01745329252f;
constexpr float kRadToDeg = 57.2957795f;

// --- lower works (static, visual + collision) ---
inline const PxVec3 kCrawlerHalf(0.45f, 0.5f, 2.6f);
constexpr float  kCrawlerOffsetX = 1.4f; // [xml] wheel x = +-1.4 (Phy.xml <wheels>)
constexpr float  kCarbodyRadius  = 1.6f;
constexpr float  kSwingHeight    = 1.25f; // top of carbody / underside of house

// --- house (upper works, swings about +Y through the origin) ---
inline const PxVec3 kHouseCenter(0.0f, 2.35f, -2.0f);
inline const PxVec3 kHouseHalf(1.7f, 1.1f, 2.8f);
inline const PxVec3 kCabCenter(-1.2f, 2.6f, 1.4f);
inline const PxVec3 kCabHalf(0.5f, 0.8f, 0.6f);
inline const PxVec3 kCounterweightCenter(0.0f, 2.0f, -5.2f);
inline const PxVec3 kCounterweightHalf(1.6f, 0.75f, 0.45f);
inline const PxVec3 kGantryTop(0.118f, 6.5f, -3.0f);

// --- boom (fixed luff for now) ---
inline const PxVec3 kBoomFoot(0.118f, 1.552f, 1.386f);       // [xml] BoomBody relativePosition
inline const PxVec3 kBoomFootToSheave(0.0f, 16.223f, 20.831f); // [xml] entity K15: sheave anchor from pivot
constexpr float  kBoomFootWidth = 0.9f;  // half-width of lattice section at foot
constexpr float  kBoomTipWidth  = 0.35f; // half-width at tip
constexpr float  kSheaveRadius  = 0.6f;

inline PxVec3 boomTipLocal() { return kBoomFoot + kBoomFootToSheave; } // y = 17.775 [xml] K12
inline PxVec3 boomAxis() { return kBoomFootToSheave.getNormalized(); }

// --- swing drive ---
constexpr float kSwingMaxRate = 0.35f; // rad/s (~3.3 rpm)
constexpr float kSwingAccel   = 0.20f; // rad/s^2

// --- hoist rope ---
constexpr float kHoistLenInitial = 8.0f;  // [xml] Dragline.HoistLenInitialM
constexpr float kHoistLenMin     = 3.5f;  // [xml] Dragline.HoistLenMinM
constexpr float kHoistLenMax     = 55.0f; // [xml] Dragline.HoistLenMaxM
constexpr float kTautBand        = 0.05f; // m, a rope/chain counts as taut within this of full length

// Hoist rope compliance (soft max-distance limit). A rigid limit chatters and blows the rig
// apart under TGS whenever the slack rope snaps taut; real wire rope stretches anyway.
constexpr float kRopeEA      = 4.0e7f; // N, axial stiffness x length: k = EA / L
constexpr float kRopeDamping = 1.5e4f; // N s/m, along the rope

// --- collision groups (filter word0 = own group, word1 = groups it collides with) ---
constexpr physx::PxU32 kGroupWorld   = 1u << 0; // ground, crawlers
constexpr physx::PxU32 kGroupMachine = 1u << 1; // house
constexpr physx::PxU32 kGroupRig     = 1u << 2; // yoke, spreader, bucket
constexpr physx::PxU32 kGroupChain   = 1u << 3; // chain links: hit everything except other links
constexpr physx::PxU32 kCollideAll   = 0xffffffffu;
constexpr physx::PxU32 kCollideChain = kGroupWorld | kGroupMachine | kGroupRig;

// --- hoist rigging: hoist rope -> yoke -> upper chains -> spreader -> lower chains -> bucket ---
constexpr float  kGroundedHeight = 0.1f; // m, bucket floor this close to the ground counts as resting on it
constexpr float  kYokeMass = 400.0f;
inline const PxVec3 kYokeHalf(0.25f, 0.2f, 0.25f);

constexpr float  kSpreaderMass = 600.0f;
constexpr float  kSpreaderHalfW = 0.8231f; // chain eyes at the bar ends, plumb over the bucket pins
inline const PxVec3 kSpreaderHalf(0.9f, 0.1f, 0.12f);

// chains: real link bodies joined by spherical joints (heavy, so they sag and lag)
constexpr float kUpperChainLen     = 2.6f;   // [xml] UpperChainLenM (yoke -> each spreader end)
constexpr int   kChainLinks        = 5;      // per chain ([xml] has 8: RopeHandlerHoistChainLinks_*_0..7); 4 lets links pass through the bucket
constexpr float kChainMassPerM     = 100.0f; // kg/m, heavy hoist chain
constexpr float kChainLinkRadius   = 0.06f;  // m, capsule radius of one link
constexpr float kChainInertiaScale = 4.0f;   // boosts link inertia: standard trick for stable joint chains

// Chain solver. Maximal: every link a rigid body, joined by iterative spherical joints.
// Articulation: one reduced-coordinate tree rooted at the bucket (bucket -> lower L -> spreader
// -> upper L -> yoke, plus upper R and lower R branches), solved exactly; only the two loop
// closures (upper R -> yoke, lower R -> spreader) and the hoist rope stay iterative joints.
// Direct: own solver (RigSolver.h) for yoke, spreader and links, joints solved exactly by sparse
// LDLT, rope and contacts as a small LCP; the PhysX bucket joins the solve through a proxy.
enum class ChainModel { Maximal, Articulation, Direct };
constexpr ChainModel   kDefaultChainModel  = ChainModel::Direct;
constexpr physx::PxU32 kArtPosIters        = 32; // 16 blows up in free fall; 48+ no better at rest
constexpr physx::PxU32 kArtVelIters        = 1;  // 4 (the spec's start value) blows up, as in the maximal rig
constexpr float        kChainTwistLimit    = 0.3f;  // rad, per link-to-link joint
constexpr float        kChainJointFriction = 0.05f; // articulation joint friction coefficient
// direct model: link-to-link friction as a rotational damper per joint. Without it a slack chain
// loop swings freely for many seconds (no contacts involved); 5 settles it, 20+ is no better.
constexpr double       kChainJointDamping  = 5.0;   // N m s/rad

// --- drag rigging (direct model only): mouth lugs -> 2 drag chains -> socket -> drag rope -> fairlead ---
// Fairlead (house-local): inside the boom foot, centred between the lattice chords, a little up the
// boom axis from the foot pin. ([xml] Dragline.Rig.FairleadLocal{X,Y,Z}M = (-0.1513, 0.5700, 3.1744)
// sits low in front of the crawlers; moved up here by choice.)
constexpr float     kFairleadAlongBoom = 1.0f; // m from the foot pin
inline const PxVec3 kFairlead = kBoomFoot + kBoomFootToSheave.getNormalized() * kFairleadAlongBoom;
constexpr float kDragLenMin       = 4.0f;  // [xml] Dragline.DragRopeLenMinM
constexpr float kDragLenMax       = 48.0f; // [xml] Dragline.DragRopeLenMaxM
constexpr float kDragInitialSlack = 0.0f;  // m of slack in the drag rope at spawn / reset (0 = just taut)
constexpr int   kDragChainLinks   = 6;     // [xml] main sim kDragChainNumLinks (14 visual slots = 2 x (6 + 1))
// not in the config (assumed until the main sim's RiggingSolver.h values are known):
constexpr float kDragChainLen       = 5.0f;   // m, each lug -> socket (3 m locks the dump: arch -> block -> socket -> chains -> lugs runs out of length near -50 deg)
constexpr float kDragChainMassPerM  = 50.0f;  // kg/m (100 + a 400 kg socket outweigh the empty bucket on the dump rope: no dump)
constexpr float kDragSocketMass     = 200.0f; // drag socket / hitch
inline const PxVec3 kDragSocketHalf(0.25f, 0.15f, 0.35f); // socket-local -Z = rope eye, toward the machine
constexpr float kDragSocketEyeHalfW = 0.25f;  // chain eyes on the socket side faces, level with its centre
// Dump rope: fixed length, bucket arch -> over the dump block -> socket top. Taut while the drag rope pulls
// the socket away: holds the mouth up; paying the drag rope out lets the bucket tip mouth-down (dump) and
// the dump rope lifts the socket toward the block.
// Dump block: a sheave (Dragline-Rigging-Diagram.png) whose two falls leave the rim kDumpSheaveRadius either
// side of its centre, not one point. It sits on the yoke at the hoist socket (yoke top), rim against the hoist
// rope on the machine side; no arm in between. Own body on a very short ball-jointed pin (frictionless: free to
// turn); the sheave axis is the yoke's. (Hung lower, below the yoke, the socket dropped into the dumped bucket.)
constexpr float     kDumpSheaveRadius = 0.3f;  // m
inline const PxVec3 kDumpBlockSheave(0.0f, kYokeHalf.y, -kDumpSheaveRadius); // yoke-local centre (-Z = machine side)
constexpr float     kDumpBlockSling = 0.05f; // m, pin -> sheave centre
inline const PxVec3 kDumpBlockHang = kDumpBlockSheave + PxVec3(0.0f, kDumpBlockSling, 0.0f); // yoke-local ball pin
constexpr float     kDumpBlockMass  = 150.0f;
inline const PxVec3 kDumpBlockHalf(0.12f, 0.18f, 0.08f);
inline const PxVec3 kSocketDumpEye(0.0f, 0.15f, 0.0f); // socket-local: top face centre
constexpr float kCarryPitchDeg = 12.0f; // mouth-up pitch the dump rope length is sized for
constexpr float kCarrySocketRiseDeg = 45.0f; // drag chains above the lug -> fairlead line in that pose: shorter dump rope
constexpr float kDumpRopeEA    = kRopeEA; // N, k = EA / L
// spawn / reset settle (direct model with the drag side): damped pre-roll, then all velocities zeroed
constexpr float kSettleTime       = 5.0f;  // s
constexpr float kSettleDt         = 1.0f / 60.0f;
constexpr float kSettleLinDamping = 2.0f;  // 1/s, extra, on every rig body and the bucket
constexpr float kSettleAngDamping = 3.0f;  // 1/s
constexpr float kSettleCarryGain  = 4.0f;  // m/s of drag rope per rad of pitch error while settling
constexpr float kSettleCarryTime  = 3.5f;  // s of the settle the servo runs; the rest settles at a fixed payout

// --- bucket (bucket-local: origin at the trunnion axis, -Z = mouth, toward the machine) ---
constexpr float  kBucketMass = 3000.0f;
inline const PxVec3 kBucketComShift(0.0f, 0.0f, -0.8f); // added to the COM of the shell shapes (cast lip + teeth are heavy)
inline const PxVec3 kHoistPin(0.8231f, -0.0235f, -0.0036f); // [xml] Dragline.Rig.HoistChainAttach{HalfW,Y,Z}M (+-X)
constexpr float  kSpreaderAboveBucket = 1.879f;             // [xml] Dragline.Rig.SpreaderBarMeshLocalYM
constexpr float  kLowerChainLen       = kSpreaderAboveBucket + 0.0235f; // spreader end -> pin, plumb
inline const PxVec3 kMouthLug(0.8917f, -0.0280f, -2.0537f); // [xml] Dragline.Rig.MouthLug{HalfW,Y,Z}M (drag chains)
inline const PxVec3 kArch(0.0f, 1.4171f, -2.0732f);          // [xml] archLocal (entity note)

// Shell: hoist pins and mouth lugs sit LOW on the side walls; the floor is just below
// the pin axis and the walls rise toward the arch over the mouth. The wall outer face is
// kept inboard of pin - link radius so the lower chains hang OUTSIDE the walls.
constexpr float kFloorBottom = -0.38f; // bucket-local y of the floor underside
constexpr float kWallTop     = 1.0f;
constexpr float kWallX       = 0.68f;  // wall centre-line
constexpr float kWallHalfT   = 0.05f;  // wall half thickness -> outer face 0.73 < 0.8231 - 0.06

struct BoxPart
{
    PxVec3 center;
    PxVec3 half;
};
constexpr float kWallMidY  = 0.5f * (kFloorBottom + kWallTop);
constexpr float kWallHalfH = 0.5f * (kWallTop - kFloorBottom);
constexpr float kInnerHalfW = kWallX - kWallHalfT;
// collision + render shapes of the bucket shell; [0] must stay the floor
inline const BoxPart kBucketParts[] = {
    {PxVec3(0.0f, kFloorBottom + 0.08f, -0.65f), PxVec3(kInnerHalfW, 0.08f, 1.55f)},                     // floor
    {PxVec3(kWallX, kWallMidY, -0.65f), PxVec3(kWallHalfT, kWallHalfH, 1.55f)},                          // wall L
    {PxVec3(-kWallX, kWallMidY, -0.65f), PxVec3(kWallHalfT, kWallHalfH, 1.55f)},                         // wall R
    {PxVec3(0.0f, kWallMidY, 0.85f), PxVec3(kInnerHalfW, kWallHalfH, 0.05f)},                            // back
    {PxVec3(kWallX, 0.5f * (kWallTop + kArch.y), kArch.z), PxVec3(kWallHalfT, 0.5f * (kArch.y - kWallTop), 0.08f)},  // arch leg L
    {PxVec3(-kWallX, 0.5f * (kWallTop + kArch.y), kArch.z), PxVec3(kWallHalfT, 0.5f * (kArch.y - kWallTop), 0.08f)}, // arch leg R
    {kArch, PxVec3(kWallX + kWallHalfT, 0.08f, 0.08f)},                                                  // arch beam
};
// Invisible covers over the cavity, direct-model rig contacts only (not PhysX, not drawn): without them
// the dump rope (no collision of its own) pulls the socket in through the open mouth of a dumped bucket
// and the drag chains follow it in and stick. No soil is simulated, so nothing else needs the cavity open.
constexpr float kFloorTop = kFloorBottom + 0.16f;
inline const BoxPart kBucketRigCovers[] = {
    {PxVec3(0.0f, 0.5f * (kFloorTop + kArch.y - 0.08f), kArch.z), PxVec3(kInnerHalfW, 0.5f * (kArch.y - 0.08f - kFloorTop), 0.05f)}, // mouth
    {PxVec3(0.0f, kWallTop - 0.05f, 0.5f * (kArch.z + 0.8f)), PxVec3(kInnerHalfW, 0.05f, 0.5f * (0.8f - kArch.z))},                 // top
};
} // namespace cfg
