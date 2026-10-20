// Headless rig check: hang -> (free fall | powered lowering) -> hoist -> swing.
// Usage: dragline_selftest [ff|lower|hang|ground|drag|carry|dump] [max|art|direct] (default ff, cfg::kDefaultChainModel).
//        dragline_selftest replay <recording.txt>: replays a recording saved in the app ([F9]).
// Exit code 1 if the rig blows up.
// hang   = no input at all for the whole run: the rig should stay put.
// ground = swing, free fall to the ground, hoist until the chains carry load with the bucket
//          still grounded, then hold: the chains should come to rest.
// drag   = (direct model) free fall to the ground, give the hoist rope 1 m of slack, pull the drag
//          rope in for 4 s, then hold: the bucket should slide toward the machine and stop.
// carry  = (direct model) from the spawn carry attitude, hoist up ~2 m (2 s) with the drag drum braked,
//          then hold: the dump rope should keep the bucket mouth up.
// dump   = (direct model) pay the drag rope out all the way (full release): the dump rope slackens and the
//          bucket tips mouth-down.
#include "Dragline.h"
#include "Recording.h"

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

// Is a world point in the open bucket cavity (between the walls, above the floor, below the wall tops)?
bool inCavity(const PxTransform& bucket, const PxVec3& p)
{
    const PxVec3 l = bucket.transformInv(p);
    return std::fabs(l.x) < cfg::kInnerHalfW && l.y > cfg::kFloorBottom + 0.16f && l.y < cfg::kWallTop &&
           l.z > -2.2f && l.z < 0.8f;
}

// drag chain links lying in the bucket cavity right now
int dragLinksInCavity(const Dragline& d)
{
    int n = 0;
    for (const Dragline::Chain c : {Dragline::DragL, Dragline::DragR})
        for (int i = 0; i < d.chainLinkCount(c); ++i)
            if (inCavity(d.bucketPose(), d.chainLinkPose(c, i).p)) ++n;
    return n;
}
} // namespace

// Replays an app recording (rec::save) step by step, then prints the final state in the same format as
// the file's "state at save" block, so the two can be diffed.
int replay(const char* path)
{
    rec::Recording r;
    if (!rec::load(path, r))
    {
        std::fprintf(stderr, "cannot read recording %s\n", path);
        return 1;
    }
    Dragline d;
    if (!d.init(r.model))
    {
        std::fprintf(stderr, "PhysX initialization failed\n");
        return 1;
    }
    std::printf("replay: %s, %zu steps (%.1f s), chains: %s\n", path, r.steps.size(), r.steps.size() * r.dt,
                rec::modelName(r.model));
    bool blewUp = false;
    for (size_t i = 0; i < r.steps.size(); ++i)
    {
        const rec::Step& s = r.steps[i];
        rec::applyTuning(d, s);
        d.step(s.controls, r.dt);
        const float t = static_cast<float>(i + 1) * r.dt;
        if (d.bucketPose().p.magnitude() > 1e4f || !d.bucketPose().p.isFinite()) blewUp = true;
        if ((i + 1) % 30 == 0 || i + 1 == r.steps.size())
        {
            std::printf("t=%6.2f in: swing %+.0f hoist %+.2f%s%s drag %+.2f | hoist pay %6.2f T %6.1fkN | pitch %+6.1f "
                        "clear %5.2f",
                        t, s.controls.swing, s.controls.hoist.lever, s.controls.hoist.freeFall ? " FF" : "",
                        s.controls.hoist.brake > 0.0f ? " BRK" : "", s.controls.drag.lever, d.drum().payout(),
                        d.lineTension() / 1000.0f, d.bucketPitch() * cfg::kRadToDeg, d.bucketClearance());
            if (d.hasDrag())
            {
                // socket relative to the dump block: horizontal distance and height below it
                const PxVec3 rel   = d.socketPose().p - d.dumpBlockPose().p;
                const float  horiz = PxVec3(rel.x, 0.0f, rel.z).magnitude();
                std::printf(" | drag pay %6.2f v %+5.2f T %6.1fkN | dump T %5.1fkN taut %d | sock from block: horiz "
                            "%5.2f below %5.2f | links in bucket %d",
                            d.dragDrum().payout(), d.dragDrum().speed(), d.dragTension() / 1000.0f,
                            d.dumpTension() / 1000.0f, d.dumpRopeTaut(), horiz, -rel.y, dragLinksInCavity(d));
            }
            std::printf("\n");
        }
    }
    std::printf("# state at the end of the replay\n");
    rec::writeState(stdout, d);
    std::printf("%s\n", blewUp ? "FAIL (blew up)" : "OK");
    d.shutdown();
    return blewUp ? 1 : 0;
}

int main(int argc, char** argv)
{
    if (argc > 2 && std::strcmp(argv[1], "replay") == 0) return replay(argv[2]);

    const bool lower = argc > 1 && std::strcmp(argv[1], "lower") == 0;
    const bool hang  = argc > 1 && std::strcmp(argv[1], "hang") == 0;
    const bool ground = argc > 1 && std::strcmp(argv[1], "ground") == 0;
    const bool drag   = argc > 1 && std::strcmp(argv[1], "drag") == 0;
    const bool carry  = argc > 1 && std::strcmp(argv[1], "carry") == 0;
    const bool dump   = argc > 1 && std::strcmp(argv[1], "dump") == 0;
    float      pitchMin = 1e9f, pitchMax = -1e9f, dumpTaut = 0.0f, dumpSamples = 0.0f; // carry / dump, after 1 s
    float      dumpedAt = -1.0f; // dump: first time the mouth points 45 deg down
    float      dumpExcessMin = 1e9f, dumpExcessMax = -1e9f; // dump rope path length minus its length (> 0 = stretched)
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
                hang ? "hang still" : lower ? "powered lowering" : ground ? "ground" : drag ? "drag"
                : carry ? "carry" : dump ? "dump" : "free fall",
                modelName);
    if ((drag || carry || dump) && !d.hasDrag())
    {
        std::fprintf(stderr, "the drag, carry and dump scenarios need the direct model (no drag side built)\n");
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

        if (carry || dump)
        {
            c = DraglineControls{};
            if (carry && t >= 1.0f && t < 3.0f) c.hoist.lever = -0.5f;
            if (dump && t >= 1.0f && t < 20.0f) c.drag.lever = 1.0f; // full release
            if (t >= 5.0f && holdStart < 0.0f) holdStart = t;
            if (t >= 1.0f)
            {
                const float pitch = d.bucketPitch() * cfg::kRadToDeg;
                pitchMin          = std::fmin(pitchMin, pitch);
                pitchMax          = std::fmax(pitchMax, pitch);
                dumpTaut += d.dumpRopeTaut() ? 1.0f : 0.0f;
                dumpSamples += 1.0f;
                if (dump && dumpedAt < 0.0f && pitch < -45.0f) dumpedAt = t - 1.0f;
            }
        }

        if (d.hasDrag()) // dump rope: one rope over the block, so arch->block + block->socket must stay <= its length
        {
            PxVec3 arch, sheave, eye;
            d.dumpEnds(arch, sheave, eye);
            const float excess = (arch - sheave).magnitude() + (eye - sheave).magnitude() - d.dumpRopeLength();
            dumpExcessMin = std::fmin(dumpExcessMin, excess);
            dumpExcessMax = std::fmax(dumpExcessMax, excess);
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
            if (d.hasDrag())
                std::printf(" | dump T=%5.1fkN rope=%d pitch=%+6.1f", d.dumpTension() / 1000.0f, d.dumpRopeTaut(),
                            d.bucketPitch() * cfg::kRadToDeg);
            if (d.hasDrag()) // socket in the bucket frame
            {
                const PxVec3 s = bucket.transformInv(d.socketPose().p);
                std::printf(" sock(%5.2f,%5.2f,%5.2f)", s.x, s.y, s.z);
            }
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
    if (carry || dump)
        std::printf("pitch bucket mouth %+.1f .. %+.1f deg after t=1 s (dump rope taut %.0f%% of it), at the end %+.1f deg, "
                    "dump tension %.1f kN\n",
                    pitchMin, pitchMax, dumpSamples > 0.0f ? 100.0f * dumpTaut / dumpSamples : 0.0f,
                    d.bucketPitch() * cfg::kRadToDeg, d.dumpTension() / 1000.0f);
    if (d.hasDrag())
        std::printf("dumpL rope length %.3f m; path - length %+.4f .. %+.4f m (> 0 stretched, < 0 slack)\n",
                    d.dumpRopeLength(), dumpExcessMin, dumpExcessMax);
    if (dump) std::printf("dump  mouth 45 deg down %.2f s after the payout started (-1 = never)\n", dumpedAt);
    if (d.hasDrag()) std::printf("cav   drag chain links lying in the bucket at the end: %d\n", dragLinksInCavity(d));
    if (ground || drag || carry || dump)
        std::printf("rest  chain links during hold (after 2 s settle): mean %.4f m/s, max %.4f m/s\n",
                    linkSamples ? linkSpeedSum / static_cast<double>(linkSamples) : -1.0, linkSpeedMax);
    if (ground || drag || carry || dump)
        std::printf("late  chain links over the last 3 s: mean %.4f m/s, max %.4f m/s\n",
                    lateSamples ? lateSpeedSum / static_cast<double>(lateSamples) : -1.0, lateSpeedMax);
    d.shutdown();
    return blewUp ? 1 : 0;
}
