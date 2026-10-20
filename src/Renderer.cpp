#include "Renderer.h"

#include "Dragline.h"
#include "DraglineConfig.h"

#include <rlgl.h>

#include <algorithm>
#include <cmath>

using namespace physx;

namespace
{
constexpr Color kSteel      = {90, 90, 98, 255};
constexpr Color kSteelDark  = {50, 50, 56, 255};
constexpr Color kBodyYellow = {222, 170, 30, 255};
constexpr Color kBodyWire   = {120, 90, 10, 255};
constexpr Color kGlass      = {120, 170, 200, 255};
constexpr Color kBoomColor  = {230, 180, 40, 255};
constexpr Color kPendant    = {170, 170, 170, 255};
constexpr Color kBucketCol  = {150, 75, 40, 255};
constexpr Color kChainCol   = {200, 200, 205, 255};
constexpr Color kChainSlack = {255, 150, 60, 255};

// draw an axis-aligned box given in the local frame of `pose`
void box(const PxTransform& pose, const PxVec3& center, const PxVec3& half, Color fill, Color wire)
{
    const PxMat44 m(pose);
    rlPushMatrix();
    rlMultMatrixf(m.front());
    DrawCube(render::toRay(center), half.x * 2.0f, half.y * 2.0f, half.z * 2.0f, fill);
    DrawCubeWires(render::toRay(center), half.x * 2.0f, half.y * 2.0f, half.z * 2.0f, wire);
    rlPopMatrix();
}

void rod(const PxVec3& a, const PxVec3& b, float radius, Color color)
{
    DrawCylinderEx(render::toRay(a), render::toRay(b), radius, radius, 6, color);
}

// Point i of n along a flexible member of given length hung between two points:
// straight when taut, otherwise a parabola with sag depth from L ~= S + 8 d^2 / (3 S).
PxVec3 memberPoint(const PxVec3& from, const PxVec3& to, float length, const PxVec3& sagHint, int i, int n)
{
    const PxVec3 span = to - from;
    const float  s    = span.magnitude();
    const float  t    = static_cast<float>(i) / static_cast<float>(n);
    const float  extra = length - s;
    if (s < 1e-3f || extra <= cfg::kTautBand) return from + span * t;

    const PxVec3 dir    = span / s;
    PxVec3       sagDir = PxVec3(0.0f, -1.0f, 0.0f) - dir * (-dir.y);
    if (sagDir.magnitude() < 0.3f) sagDir = sagHint - dir * sagHint.dot(dir); // near-vertical member
    sagDir.normalize();
    const float sag = std::min(std::sqrt(3.0f * s * extra / 8.0f), 0.5f * length);
    return from + span * t + sagDir * (4.0f * sag * t * (1.0f - t));
}
} // namespace

namespace render
{
void ground()
{
    DrawPlane({0.0f, 0.0f, 0.0f}, {120.0f, 120.0f}, {82, 74, 60, 255});
    DrawGrid(60, 2.0f);
}

void lowerWorks()
{
    for (const float side : {-1.0f, 1.0f})
    {
        const PxVec3 c(side * cfg::kCrawlerOffsetX, cfg::kCrawlerHalf.y, 0.0f);
        box(PxTransform(PxIdentity), c, cfg::kCrawlerHalf, kSteelDark, BLACK);
        for (float z = -cfg::kCrawlerHalf.z; z <= cfg::kCrawlerHalf.z; z += 0.5f) // track pads
        {
            DrawLine3D({c.x - cfg::kCrawlerHalf.x, 0.02f, z}, {c.x + cfg::kCrawlerHalf.x, 0.02f, z}, BLACK);
        }
    }
    const float tubBottom = cfg::kCrawlerHalf.y * 2.0f;
    const float tubHeight = cfg::kSwingHeight - tubBottom;
    DrawCylinder({0.0f, tubBottom, 0.0f}, cfg::kCarbodyRadius, cfg::kCarbodyRadius, tubHeight, 24, kSteel);
    DrawCylinderWires({0.0f, tubBottom, 0.0f}, cfg::kCarbodyRadius, cfg::kCarbodyRadius, tubHeight, 24, kSteelDark);
}

void house(const PxTransform& pose)
{
    box(pose, cfg::kHouseCenter, cfg::kHouseHalf, kBodyYellow, kBodyWire);
    box(pose, cfg::kCabCenter, cfg::kCabHalf, kGlass, kSteelDark);
    box(pose, cfg::kCounterweightCenter, cfg::kCounterweightHalf, kSteelDark, BLACK);

    // gantry (A-frame): front and back leg on each side, meeting at the top cross-beam
    const float  deckY = cfg::kHouseCenter.y + cfg::kHouseHalf.y;
    const float  backZ = cfg::kHouseCenter.z - cfg::kHouseHalf.z + 0.4f;
    const float  frontZ = cfg::kHouseCenter.z + cfg::kHouseHalf.z - 1.6f;
    const float  legX  = cfg::kHouseHalf.x - 0.3f;
    for (const float side : {-1.0f, 1.0f})
    {
        const PxVec3 topSide = pose.transform(cfg::kGantryTop + PxVec3(side * 0.5f, 0.0f, 0.0f));
        rod(pose.transform(PxVec3(side * legX, deckY, backZ)), topSide, 0.1f, kBoomColor);
        rod(pose.transform(PxVec3(side * legX, deckY, frontZ)), topSide, 0.1f, kBoomColor);
    }
    rod(pose.transform(cfg::kGantryTop + PxVec3(-0.5f, 0.0f, 0.0f)),
        pose.transform(cfg::kGantryTop + PxVec3(0.5f, 0.0f, 0.0f)), 0.1f, kBoomColor);
}

void boom(const PxTransform& housePose)
{
    // lattice section: square, tapering from foot to tip
    const PxVec3 axis = cfg::boomAxis();
    const PxVec3 side(1.0f, 0.0f, 0.0f);
    const PxVec3 up   = axis.cross(side); // perpendicular to boom, pointing upward
    const PxVec3 foot = cfg::kBoomFoot;
    const PxVec3 tip  = cfg::boomTipLocal();

    const float corners[4][2] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}};
    auto corner = [&](int i, float t) {
        const float  w = cfg::kBoomFootWidth + (cfg::kBoomTipWidth - cfg::kBoomFootWidth) * t;
        const PxVec3 c = foot + (tip - foot) * t + side * (corners[i][0] * w) + up * (corners[i][1] * w);
        return housePose.transform(c);
    };

    for (int i = 0; i < 4; ++i) rod(corner(i, 0.0f), corner(i, 1.0f), 0.07f, kBoomColor);

    constexpr int kBays = 13;
    for (int k = 0; k <= kBays; ++k)
    {
        const float t0 = static_cast<float>(k) / kBays;
        for (int i = 0; i < 4; ++i)
        {
            DrawLine3D(toRay(corner(i, t0)), toRay(corner((i + 1) % 4, t0)), kBodyWire); // ring
            if (k < kBays)
            {
                const float t1 = static_cast<float>(k + 1) / kBays;
                DrawLine3D(toRay(corner(i, t0)), toRay(corner((i + 1) % 4, t1)), kBodyWire); // lacing
            }
        }
    }

    // foot pin and tip sheave
    rod(housePose.transform(foot - side * 1.1f), housePose.transform(foot + side * 1.1f), 0.15f, kSteel);
    const PxVec3 tipW  = housePose.transform(tip);
    const PxVec3 sideW = housePose.q.rotate(side);
    DrawCylinderEx(toRay(tipW - sideW * 0.15f), toRay(tipW + sideW * 0.15f), cfg::kSheaveRadius, cfg::kSheaveRadius,
                   16, kSteel);
    DrawCylinderWiresEx(toRay(tipW - sideW * 0.15f), toRay(tipW + sideW * 0.15f), cfg::kSheaveRadius,
                        cfg::kSheaveRadius, 16, kSteelDark);

    // pendant ropes: gantry top -> boom tip (both sides)
    const PxVec3 gantry = housePose.transform(cfg::kGantryTop);
    for (const float s : {-1.0f, 1.0f})
    {
        const PxVec3 anchor = housePose.transform(tip + side * (s * cfg::kBoomTipWidth) + up * cfg::kBoomTipWidth);
        DrawLine3D(toRay(gantry + sideW * (s * 0.5f)), toRay(anchor), kPendant);
    }
}

void rope(const PxVec3& from, const PxVec3& to, float length, const PxVec3& sagHint, Color color)
{
    constexpr int kSegments = 24;
    PxVec3        prev      = from;
    for (int i = 1; i <= kSegments; ++i)
    {
        const PxVec3 p = memberPoint(from, to, length, sagHint, i, kSegments);
        rod(prev, p, 0.04f, color);
        prev = p;
    }
}

void chainLink(const PxTransform& pose, float length, int index, Color color)
{
    // capsule matching the physics shape; every other link a shade darker so the links read
    const float  r = cfg::kChainLinkRadius;
    const PxVec3 a = pose.transform(PxVec3(-0.5f * length + r, 0.0f, 0.0f));
    const PxVec3 b = pose.transform(PxVec3(0.5f * length - r, 0.0f, 0.0f));
    const Color  c = (index % 2) ? ColorBrightness(color, -0.25f) : color;
    DrawCapsule(toRay(a), toRay(b), r, 6, 3, c);
}

void yoke(const PxTransform& pose)
{
    box(pose, PxVec3(0.0f), cfg::kYokeHalf, kSteel, BLACK);
}

void spreader(const PxTransform& pose)
{
    box(pose, PxVec3(0.0f), cfg::kSpreaderHalf, kBodyYellow, kBodyWire);
}

void dragSocket(const PxTransform& pose)
{
    box(pose, PxVec3(0.0f), cfg::kDragSocketHalf, kSteelDark, BLACK);
}

void bucket(const PxTransform& pose)
{
    for (const cfg::BoxPart& part : cfg::kBucketParts) box(pose, part.center, part.half, kBucketCol, BLACK);

    // hoist trunnion pins (chain attach) and drag mouth lugs: stubs out from the wall face
    const float wallFace = cfg::kWallX + cfg::kWallHalfT;
    for (const float s : {-1.0f, 1.0f})
    {
        const PxVec3 pin(s * cfg::kHoistPin.x, cfg::kHoistPin.y, cfg::kHoistPin.z);
        rod(pose.transform(PxVec3(s * wallFace, pin.y, pin.z)), pose.transform(pin + PxVec3(s * 0.08f, 0.0f, 0.0f)),
            0.08f, kSteel);
        const PxVec3 lug(s * cfg::kMouthLug.x, cfg::kMouthLug.y, cfg::kMouthLug.z);
        rod(pose.transform(PxVec3(s * wallFace, lug.y, lug.z)), pose.transform(lug + PxVec3(s * 0.06f, 0.0f, 0.0f)),
            0.07f, kSteelDark);
    }

    // teeth along the lip
    const cfg::BoxPart& floor = cfg::kBucketParts[0];
    const float         lipZ  = floor.center.z - floor.half.z;
    for (int i = 0; i < 5; ++i)
    {
        const float x = -0.6f + 0.3f * static_cast<float>(i);
        box(pose, PxVec3(x, floor.center.y, lipZ - 0.15f), PxVec3(0.06f, 0.05f, 0.15f), kSteelDark, BLACK);
    }
}

void dragline(const Dragline& d)
{
    const PxTransform hp      = d.housePose();
    const PxVec3      forward = hp.q.rotate(PxVec3(0.0f, 0.0f, 1.0f));
    lowerWorks();
    house(hp);
    boom(hp);

    // hoist rope: along the boom from the foot -> over the tip sheave -> yoke
    PxVec3 tip, yokeTop;
    d.hoistEnds(tip, yokeTop);
    rope(hp.transform(cfg::kBoomFoot), tip, 0.0f, forward, DARKGRAY);
    rope(tip, yokeTop, d.drum().payout(), forward, d.ropeTaut() ? Color{240, 220, 120, 255} : ORANGE);

    for (int c = 0; c < Dragline::ChainCount; ++c)
    {
        const auto  chainId = static_cast<Dragline::Chain>(c);
        const Color color   = d.chainTaut(chainId) ? kChainCol : kChainSlack;
        for (int i = 0; i < d.chainLinkCount(chainId); ++i)
            chainLink(d.chainLinkPose(chainId, i), d.chainLinkLength(chainId), i, color);
    }

    // drag rope: fairlead sheave on the house front -> socket
    if (d.hasDrag())
    {
        PxVec3 fairlead, socketEye;
        d.dragEnds(fairlead, socketEye);
        const PxVec3 sideW = hp.q.rotate(PxVec3(1.0f, 0.0f, 0.0f));
        DrawCylinderEx(toRay(fairlead - sideW * 0.12f), toRay(fairlead + sideW * 0.12f), 0.3f, 0.3f, 12, kSteel);
        rope(fairlead, socketEye, d.dragDrum().payout(), forward,
             d.dragRopeTaut() ? Color{240, 220, 120, 255} : ORANGE);
        dragSocket(d.socketPose());

        // dump rope: arch -> over the dump block sheave (yoke) -> socket top; slack shared by segment length
        PxVec3 arch, sheave, dumpEye;
        d.dumpEnds(arch, sheave, dumpEye);
        const float l1    = (arch - sheave).magnitude();
        const float l2    = (dumpEye - sheave).magnitude();
        const float slack = std::max(d.dumpRopeLength() - l1 - l2, 0.0f);
        const float share = l1 + l2 > 1e-3f ? l1 / (l1 + l2) : 0.5f;
        const Color dumpCol = d.dumpRopeTaut() ? Color{240, 220, 120, 255} : ORANGE;
        rope(arch, sheave, l1 + slack * share, forward, dumpCol);
        rope(sheave, dumpEye, l2 + slack * (1.0f - share), forward, dumpCol);
        const PxVec3 yokeSide = d.yokePose().q.rotate(PxVec3(0.08f, 0.0f, 0.0f));
        DrawCylinderEx(toRay(sheave - yokeSide), toRay(sheave + yokeSide), 0.18f, 0.18f, 12, kSteelDark);
    }

    yoke(d.yokePose());
    spreader(d.spreaderPose());
    bucket(d.bucketPose());
}
} // namespace render
