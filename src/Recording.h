#pragma once

#include "Dragline.h"

#include <cstdio>
#include <string>
#include <vector>

// Input recording for bug hunting: the app records every physics step's inputs since the last reset
// and saves them on request (with a snapshot of the rig state at that moment); the self-test replays
// them headless (dragline_selftest replay <file>) and prints the same snapshot to compare.
namespace rec
{
struct Step
{
    DraglineControls controls;
    float            swingRpm    = 0.0f; // slider values in effect for this step
    float            hoistPullKN = 0.0f;
    float            dragPullKN  = 0.0f;
};

struct Recording
{
    cfg::ChainModel   model = cfg::kDefaultChainModel;
    float             dt    = 1.0f / 60.0f;
    std::vector<Step> steps;
};

inline FILE* openFile(const char* path, const char* mode)
{
    FILE* f = nullptr;
    return fopen_s(&f, path, mode) == 0 ? f : nullptr;
}

inline const char* modelName(cfg::ChainModel m)
{
    return m == cfg::ChainModel::Maximal ? "max" : m == cfg::ChainModel::Articulation ? "art" : "direct";
}

// slider values -> rig (the app does this when a slider moves; replay does it every step)
inline void applyTuning(Dragline& d, const Step& s)
{
    d.setSwingMaxRate(s.swingRpm * 2.0f * 3.14159265f / 60.0f);
    d.setHoistMotorForce(s.hoistPullKN * 1000.0f);
    d.setDragMotorForce(s.dragPullKN * 1000.0f);
}

inline void writePose(FILE* f, const char* name, const physx::PxTransform& p)
{
    std::fprintf(f, "%-10s p %.6f %.6f %.6f  q %.6f %.6f %.6f %.6f\n", name, p.p.x, p.p.y, p.p.z, p.q.x, p.q.y, p.q.z,
                 p.q.w);
}

// everything visible through the public API, full precision so two runs can be diffed
inline void writeState(FILE* f, const Dragline& d)
{
    std::fprintf(f, "swing      angle %.6f rate %.6f max %.6f\n", d.swingAngle(), d.swingRate(), d.swingMaxRate());
    std::fprintf(f, "hoist      payout %.6f speed %.6f tension %.3f taut %d motor %.3f\n", d.drum().payout(),
                 d.drum().speed(), d.lineTension(), d.ropeTaut(), d.drum().motorForce());
    if (d.hasDrag())
    {
        std::fprintf(f, "drag       payout %.6f speed %.6f tension %.3f taut %d motor %.3f\n", d.dragDrum().payout(),
                     d.dragDrum().speed(), d.dragTension(), d.dragRopeTaut(), d.dragDrum().motorForce());
        std::fprintf(f, "dumprope   length %.6f tension %.3f taut %d\n", d.dumpRopeLength(), d.dumpTension(),
                     d.dumpRopeTaut());
    }
    std::fprintf(f, "bucket     pitch %.4f deg clearance %.6f\n", d.bucketPitch() * cfg::kRadToDeg, d.bucketClearance());
    writePose(f, "bucket", d.bucketPose());
    writePose(f, "yoke", d.yokePose());
    writePose(f, "spreader", d.spreaderPose());
    if (d.hasDrag()) writePose(f, "socket", d.socketPose());
    for (int c = 0; c < Dragline::ChainCount; ++c)
    {
        const auto chain = static_cast<Dragline::Chain>(c);
        for (int i = 0; i < d.chainLinkCount(chain); ++i)
        {
            char name[16];
            std::snprintf(name, sizeof(name), "link%d.%d", c, i);
            writePose(f, name, d.chainLinkPose(chain, i));
        }
    }
}

inline bool save(const std::string& path, const Recording& r, const Dragline& d)
{
    FILE* f = openFile(path.c_str(), "w");
    if (!f) return false;
    std::fprintf(f, "# customDragline recording v1\nmodel %s\ndt %.9f\nsteps %zu\n", modelName(r.model), r.dt,
                 r.steps.size());
    std::fprintf(f, "# swing hoistLever hoistFreeFall hoistBrake dragLever swingRpm hoistPullKN dragPullKN\n");
    for (const Step& s : r.steps)
        std::fprintf(f, "%.9g %.9g %d %.9g %.9g %.9g %.9g %.9g\n", s.controls.swing, s.controls.hoist.lever,
                     s.controls.hoist.freeFall ? 1 : 0, s.controls.hoist.brake, s.controls.drag.lever, s.swingRpm,
                     s.hoistPullKN, s.dragPullKN);
    std::fprintf(f, "# state at save (after the last step)\n");
    writeState(f, d);
    std::fclose(f);
    return true;
}

inline bool load(const std::string& path, Recording& r)
{
    FILE* f = openFile(path.c_str(), "r");
    if (!f) return false;
    char   model[16] = {};
    size_t n         = 0;
    bool   ok        = fscanf_s(f, "# customDragline recording v1 model %15s dt %f steps %zu", model, static_cast<unsigned>(sizeof(model)), &r.dt, &n) == 3;
    if (ok)
    {
        r.model = std::string(model) == "max" ? cfg::ChainModel::Maximal
                : std::string(model) == "art" ? cfg::ChainModel::Articulation
                                              : cfg::ChainModel::Direct;
        char line[256];
        std::fgets(line, sizeof(line), f); // rest of the steps line
        std::fgets(line, sizeof(line), f); // column header
        r.steps.resize(n);
        for (Step& s : r.steps)
        {
            int freeFall = 0;
            if (fscanf_s(f, "%f %f %d %f %f %f %f %f", &s.controls.swing, &s.controls.hoist.lever, &freeFall,
                            &s.controls.hoist.brake, &s.controls.drag.lever, &s.swingRpm, &s.hoistPullKN,
                            &s.dragPullKN) != 8)
            {
                ok = false;
                break;
            }
            s.controls.hoist.freeFall = freeFall != 0;
        }
    }
    std::fclose(f);
    return ok;
}
} // namespace rec
