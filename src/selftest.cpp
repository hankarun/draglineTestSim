// Headless rig check: hang -> (free fall | powered lowering) -> hoist -> swing.
// Usage: dragline_selftest [ff|lower|hang|ground|drag] [max|art|direct] (default ff, cfg::kDefaultChainModel).
// Exit code 1 if the rig blows up.
// hang   = no input at all for the whole run: the rig should stay put.
// ground = swing, free fall to the ground, hoist until the chains carry load with the bucket
//          still grounded, then hold: the chains should come to rest.
// drag   = (direct model) free fall to the ground, give the hoist rope 1 m of slack, pull the drag
//          rope in for 4 s, then hold: the bucket should slide toward the machine and stop.
#include "Dragline.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

using namespace physx;

namespace
{
constexpr float kDt       = 1.0f / 60.0f;
constexpr float kDuration = 24.0f;
constexpr float kBlowUpSpeed = 30.0f; // m/s, nothing in this rig should move this fast

// Is a world point inside any bucket shell box?
bool insideBucket(const PxTransform& bucket, const PxVec3& p)
{
    const PxVec3 local = bucket.transformInv(p);
    for (const cfg::BoxPart& part : cfg::kBucketParts)
    {
        const PxVec3 d = local - part.center;
        if (std::fabs(d.x) < part.half.x && std::fabs(d.y) < part.half.y && std::fabs(d.z) < part.half.z) return true;
    }
    return false;
}
} // namespace

int main(int argc, char** argv)
{
    const bool lower = argc > 1 && std::strcmp(argv[1], "lower") == 0;
    const bool hang  = argc > 1 && std::strcmp(argv[1], "hang") == 0;
    const bool ground = argc > 1 && std::strcmp(argv[1], "ground") == 0;
    const bool drag   = argc > 1 && std::strcmp(argv[1], "drag") == 0;
    enum class Phase { Swing, Drop, Brake, Hoist, Hold } phase = Phase::Swing;
    enum class DragPhase { Settle, Drop, Brake, Slack, Pull, Hold } dragPhase = DragPhase::Settle;
    float  phaseStart = 0.0f, pullStartZ = 0.0f, pullEndZ = 0.0f;
    double dragTensionSum = 0.0;
    long   dragTensionSamples = 0;
    float  dragTensionMax = 0.0f;
    float holdStart = -1.0f;
    double linkSpeedSum = 0.0;
    float  linkSpeedMax = 0.0f;
    long   linkSamples  = 0;
    double lateSpeedSum = 0.0; // same over the last 3 s of the run: has the rig come to rest at all?
    float  lateSpeedMax = 0.0f;
    long   lateSamples  = 0;

    cfg::ChainModel model = cfg::kDefaultChainModel;
    if (argc > 2 && std::strcmp(argv[2], "max") == 0) model = cfg::ChainModel::Maximal;
    if (argc > 2 && std::strcmp(argv[2], "art") == 0) model = cfg::ChainModel::Articulation;
    if (argc > 2 && std::strcmp(argv[2], "direct") == 0) model = cfg::ChainModel::Direct;
    const char* modelName = model == cfg::ChainModel::Maximal ? "maximal"
                          : model == cfg::ChainModel::Articulation ? "articulation"
                                                                    : "direct";
    float      maxEndGap = 0.0f; // worst chain-end joint separation (loop closures in the articulation model)

    Dragline d;
    if (!d.init(model))
    {
        std::fprintf(stderr, "PhysX initialization failed\n");
        return 1;
    }

    bool  blewUp      = false;
    int   penetrating = 0;
    float maxSpeed    = 0.0f;
    std::printf("scenario: %s, chains: %s\n",
                hang ? "hang still" : lower ? "powered lowering" : ground ? "ground" : drag ? "drag" : "free fall", modelName);
    if (drag && !d.hasDrag())
    {
        std::fprintf(stderr, "the drag scenario needs the direct model (no drag side built)\n");
        return 1;
    }
    double stepMs = 0.0, worstMs = 0.0;
    float  swayMin = 1e9f, swayMax = -1e9f; // yoke fore/aft travel while nothing is commanded

    DraglineControls c;
    for (int i = 0; i <= static_cast<int>(kDuration / kDt); ++i)
    {
        const float t    = static_cast<float>(i) * kDt;
        c.hoist.freeFall = !lower && !hang && t >= 4.0f && t < 14.0f;
        c.hoist.brake    = 0.0f;
        c.hoist.lever    = hang ? 0.0f
                         : (t >= 14.0f && t < 17.0f)    ? -1.0f
                         : (lower && t >= 4.0f && t < 7.0f) ? 0.5f
                                                            : 0.0f;
        c.swing = (!hang && t >= 18.0f && t < 21.0f) ? 1.0f : 0.0f;
        if (ground)
        {
            c = DraglineControls{};
            switch (phase)
            {
            case Phase::Swing: // swing left for 3 s, coast to a stop
                c.swing = t < 3.0f ? 1.0f : 0.0f;
                if (t >= 5.0f) phase = Phase::Drop;
                break;
            case Phase::Drop: // free fall until the bucket lands
                c.hoist.freeFall = true;
                if (d.bucketClearance() < 0.05f || !d.ropeTaut()) phase = Phase::Brake;
                break;
            case Phase::Brake: // stop the overrunning drum
                c.hoist.freeFall = true;
                c.hoist.brake    = 1.0f;
                if (d.drum().speed() == 0.0f) phase = Phase::Hoist;
                break;
            case Phase::Hoist: // reel in slowly until the rope takes the yoke + chains + spreader
                c.hoist.lever = -0.3f;
                if (d.ropeTaut() && d.lineTension() > 12000.0f)
                {
                    phase     = Phase::Hold;
                    holdStart = t;
                    std::printf("-- hold from t=%.2f, tension %.1f kN, clearance %.2f\n", t, d.lineTension() / 1000.0f,
                                d.bucketClearance());
                }
                break;
            case Phase::Hold:
                break;
            }
        }
        if (drag)
        {
            c = DraglineControls{};
            auto next = [&](DragPhase p) {
                dragPhase  = p;
                phaseStart = t;
            };
            switch (dragPhase)
            {
            case DragPhase::Settle:
                if (t >= 1.0f) next(DragPhase::Drop);
                break;
            case DragPhase::Drop:
                c.hoist.freeFall = true;
                if (d.bucketClearance() < 0.05f || !d.ropeTaut()) next(DragPhase::Brake);
                break;
            case DragPhase::Brake:
                c.hoist.freeFall = true;
                c.hoist.brake    = 1.0f;
                if (d.drum().speed() == 0.0f) next(DragPhase::Slack);
                break;
            case DragPhase::Slack: // 1 m of hoist slack so the bucket can be dragged out from under the boom tip
                c.hoist.lever = 0.5f;
                if (t - phaseStart >= 1.0f)
                {
                    next(DragPhase::Pull);
                    pullStartZ = d.bucketPose().p.z;
                    std::printf("-- pull from t=%.2f, bucket z %.2f\n", t, pullStartZ);
                }
                break;
            case DragPhase::Pull:
                c.drag.lever = -0.5f;
                if (d.chainTaut(Dragline::DragL) && d.chainTaut(Dragline::DragR) && d.dragTension() > 1000.0f)
                {
                    dragTensionSum += d.dragTension();
                    ++dragTensionSamples;
                }
                dragTensionMax = std::fmax(dragTensionMax, d.dragTension());
                if (t - phaseStart >= 4.0f)
                {
                    next(DragPhase::Hold);
                    pullEndZ  = d.bucketPose().p.z;
                    holdStart = t;
                    std::printf("-- hold from t=%.2f, bucket z %.2f\n", t, pullEndZ);
                }
                break;
            case DragPhase::Hold:
                break;
            }
        }

        const auto t0 = std::chrono::steady_clock::now();
        d.step(c, kDt);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        stepMs += ms;
        worstMs = ms > worstMs ? ms : worstMs;
        if (hang || t < 4.0f)
        {
            swayMin = std::fmin(swayMin, d.yokePose().p.z);
            swayMax = std::fmax(swayMax, d.yokePose().p.z);
        }

        // blow-up and penetration checks over every rig body
        const PxTransform bucket = d.bucketPose();
        float             frameMax = 0.0f;
        auto track = [&](const PxVec3& prev, const PxVec3& now) {
            frameMax = std::fmax(frameMax, (now - prev).magnitude() / kDt);
        };
        static PxVec3 prevPos[64];
        int           k = 0;
        for (const PxVec3& p : {d.yokePose().p, d.spreaderPose().p, bucket.p}) { if (i) track(prevPos[k], p); prevPos[k++] = p; }
        if (insideBucket(bucket, d.spreaderPose().p)) ++penetrating;
        if (d.hasDrag())
        {
            if (i) track(prevPos[k], d.socketPose().p);
            prevPos[k++] = d.socketPose().p;
            if (insideBucket(bucket, d.socketPose().p)) ++penetrating;
        }
        for (int ch = 0; ch < Dragline::ChainCount; ++ch)
        {
            const auto chain = static_cast<Dragline::Chain>(ch);
            const int  n     = d.chainLinkCount(chain);
            if (n == 0) continue;
            const PxVec3 half(0.5f * d.chainLinkLength(chain), 0.0f, 0.0f);
            PxVec3 eyeA, eyeB;
            d.chainEnds(chain, eyeA, eyeB);
            maxEndGap = std::fmax(maxEndGap, (d.chainLinkPose(chain, 0).transform(-half) - eyeA).magnitude());
            maxEndGap = std::fmax(maxEndGap, (d.chainLinkPose(chain, n - 1).transform(half) - eyeB).magnitude());
        }
        for (int ch = 0; ch < Dragline::ChainCount; ++ch)
            for (int l = 0; l < d.chainLinkCount(static_cast<Dragline::Chain>(ch)); ++l)
            {
                const PxVec3 p = d.chainLinkPose(static_cast<Dragline::Chain>(ch), l).p;
                if (i && holdStart >= 0.0f && t > holdStart + 2.0f) // settle time, then the links should be at rest
                {
                    const float s = (p - prevPos[k]).magnitude() / kDt;
                    linkSpeedSum += s;
                    linkSpeedMax = std::fmax(linkSpeedMax, s);
                    ++linkSamples;
                    if (t > kDuration - 3.0f)
                    {
                        lateSpeedSum += s;
                        lateSpeedMax = std::fmax(lateSpeedMax, s);
                        ++lateSamples;
                    }
                }
                if (i) track(prevPos[k], p);
                prevPos[k++] = p;
                if (insideBucket(bucket, p)) ++penetrating;
            }
        maxSpeed = std::fmax(maxSpeed, frameMax);
        if (frameMax > kBlowUpSpeed) blewUp = true;

        if (i % 30 == 0)
        {
            std::printf("t=%5.2f pay=%6.2f v=%+5.2f T=%6.1fkN bucket(%6.2f,%5.2f,%6.2f) clear=%5.2f rope=%d "
                        "chains=%d%d%d%d vmax=%5.2f pen=%d",
                        t, d.drum().payout(), d.drum().speed(), d.lineTension() / 1000.0f, bucket.p.x, bucket.p.y,
                        bucket.p.z, d.bucketClearance(), d.ropeTaut(), d.chainTaut(Dragline::UpperL),
                        d.chainTaut(Dragline::UpperR), d.chainTaut(Dragline::LowerL), d.chainTaut(Dragline::LowerR),
                        frameMax, penetrating);
            if (d.hasDrag())
                std::printf(" | drag pay=%6.2f v=%+5.2f T=%6.1fkN rope=%d chains=%d%d", d.dragDrum().payout(),
                            d.dragDrum().speed(), d.dragTension() / 1000.0f, d.dragRopeTaut(),
                            d.chainTaut(Dragline::DragL), d.chainTaut(Dragline::DragR));
            std::printf("\n");
        }
    }

    const double steps = kDuration / kDt + 1.0;
    std::printf("%s  max body speed %.2f m/s, penetrating samples %d\n", blewUp ? "FAIL (blew up)" : "OK", maxSpeed,
                penetrating);
    std::printf("perf  %.2f ms per 1/60 s step (worst %.2f) -> %.1fx realtime\n", stepMs / steps, worstMs,
                16.667 / (stepMs / steps));
    std::printf("sway  yoke fore/aft travel with no input: %.3f m\n", swayMax - swayMin);
    std::printf("gap   worst chain-end joint separation: %.4f m\n", maxEndGap);
    if (drag)
        std::printf("drag  bucket moved %.2f m toward the machine; taut-chain drag tension mean %.1f kN, max %.1f kN "
                    "(mu 0.5 x bucket weight = %.1f kN)\n",
                    pullStartZ - pullEndZ, dragTensionSamples ? dragTensionSum / dragTensionSamples / 1000.0 : 0.0,
                    dragTensionMax / 1000.0f, 0.5f * cfg::kBucketMass * cfg::kGravity / 1000.0f);
    if (ground || drag)
        std::printf("rest  chain links during hold (after 2 s settle): mean %.4f m/s, max %.4f m/s\n",
                    linkSamples ? linkSpeedSum / static_cast<double>(linkSamples) : -1.0, linkSpeedMax);
    if (ground || drag)
        std::printf("late  chain links over the last 3 s: mean %.4f m/s, max %.4f m/s\n",
                    lateSamples ? lateSpeedSum / static_cast<double>(lateSamples) : -1.0, lateSpeedMax);
    d.shutdown();
    return blewUp ? 1 : 0;
}
