#include "Dragline.h"
#include "Renderer.h"

#include <raylib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

using namespace physx;

namespace
{
constexpr float kFixedDt     = 1.0f / 60.0f; // physics step (s)
constexpr int   kMaxSubSteps = 5;            // avoid spiral of death

struct OrbitCamera
{
    float    yaw      = 0.9f;
    float    pitch    = 0.35f;
    float    distance = 60.0f;
    Vector3  target   = {0.0f, 8.0f, 0.0f};
    Camera3D camera{};

    OrbitCamera()
    {
        camera.up         = {0.0f, 1.0f, 0.0f};
        camera.fovy       = 45.0f;
        camera.projection = CAMERA_PERSPECTIVE;
        update();
    }

    void update()
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
        {
            const Vector2 d = GetMouseDelta();
            yaw -= d.x * 0.005f;
            pitch = std::clamp(pitch + d.y * 0.005f, 0.02f, 1.5f);
        }
        distance = std::clamp(distance * std::pow(0.9f, GetMouseWheelMove()), 5.0f, 250.0f);

        camera.target   = target;
        camera.position = {target.x + distance * std::cos(pitch) * std::sin(yaw),
                           target.y + distance * std::sin(pitch),
                           target.z + distance * std::cos(pitch) * std::cos(yaw)};
    }
};

float axis(int positiveKey, int negativeKey)
{
    return (IsKeyDown(positiveKey) ? 1.0f : 0.0f) - (IsKeyDown(negativeKey) ? 1.0f : 0.0f);
}
} // namespace

int main(int argc, char** argv)
{
    // optional chain model: customDragline [max|art|direct]
    cfg::ChainModel model = cfg::kDefaultChainModel;
    if (argc > 1 && std::strcmp(argv[1], "max") == 0) model = cfg::ChainModel::Maximal;
    if (argc > 1 && std::strcmp(argv[1], "art") == 0) model = cfg::ChainModel::Articulation;
    if (argc > 1 && std::strcmp(argv[1], "direct") == 0) model = cfg::ChainModel::Direct;
    const char* modelName = model == cfg::ChainModel::Maximal ? "maximal (PhysX joints)"
                          : model == cfg::ChainModel::Articulation ? "articulation (PhysX)"
                                                                    : "direct (own LDLT solver)";

    Dragline dragline;
    if (!dragline.init(model))
    {
        std::fprintf(stderr, "PhysX initialization failed\n");
        dragline.shutdown();
        return 1;
    }

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "customDragline - raylib + PhysX");

    OrbitCamera orbit;
    float       accumulator = 0.0f;
    bool        paused      = false;
    bool        freeFall    = false;

    while (!WindowShouldClose())
    {
        // --- input ---
        if (IsKeyPressed(KEY_R)) { dragline.reset(); accumulator = 0.0f; freeFall = false; }
        if (IsKeyPressed(KEY_P)) paused = !paused;
        if (IsKeyPressed(KEY_F)) freeFall = !freeFall;
        orbit.update();

        DraglineControls controls;
        controls.swing          = axis(KEY_A, KEY_D);
        controls.hoist.lever    = axis(KEY_S, KEY_W);
        controls.hoist.freeFall = freeFall;
        controls.hoist.brake    = IsKeyDown(KEY_SPACE) ? 1.0f : 0.0f;
        controls.drag.lever     = axis(KEY_E, KEY_Q); // Q pulls the drag rope in, E pays it out

        // --- fixed-step physics ---
        if (!paused)
        {
            accumulator += GetFrameTime();
            accumulator = std::min(accumulator, kFixedDt * kMaxSubSteps);
            while (accumulator >= kFixedDt)
            {
                dragline.step(controls, kFixedDt);
                accumulator -= kFixedDt;
            }
        }

        // --- render ---
        BeginDrawing();
        ClearBackground({150, 180, 210, 255});

        BeginMode3D(orbit.camera);
        render::ground();
        render::dragline(dragline);
        EndMode3D();

        const HoistDrum& drum = dragline.drum();
        const char* brake = freeFall ? (controls.hoist.brake > 0.0f ? "OPERATOR BRAKE" : "released")
                                     : (drum.holdingBrake() ? "HOLDING" : "released");
        int y = 10;
        auto line = [&](const char* text, Color c = RAYWHITE) {
            DrawText(text, 11, y + 1, 20, BLACK);
            DrawText(text, 10, y, 20, c);
            y += 24;
        };
        line(TextFormat("swing   %7.1f deg   %+5.2f rpm", dragline.swingAngle() * cfg::kRadToDeg,
                        dragline.swingRate() * 60.0f / (2.0f * PI)));
        line(TextFormat("hoist   payout %6.2f m   line speed %+5.2f m/s (+ = lowering)", drum.payout(), drum.speed()));
        line(freeFall ? "mode    FREE FALL (clutch out)" : "mode    POWERED", freeFall ? ORANGE : RAYWHITE);
        line(TextFormat("brake   %s   motor %6.1f kN", brake, drum.motorForce() / 1000.0f));
        line(TextFormat("rope    %s   tension %6.1f kN", dragline.ropeTaut() ? "TAUT " : "SLACK",
                        dragline.lineTension() / 1000.0f),
             dragline.ropeTaut() ? RAYWHITE : ORANGE);
        auto chainState = [&](Dragline::Chain c) { return dragline.chainTaut(c) ? "taut " : "SLACK"; };
        line(TextFormat("chains  upper L %s R %s   lower L %s R %s", chainState(Dragline::UpperL),
                        chainState(Dragline::UpperR), chainState(Dragline::LowerL), chainState(Dragline::LowerR)));
        if (dragline.hasDrag())
        {
            const HoistDrum& dragDrum = dragline.dragDrum();
            line(TextFormat("drag    payout %6.2f m   line speed %+5.2f m/s   %s   tension %6.1f kN", dragDrum.payout(),
                            dragDrum.speed(), dragline.dragRopeTaut() ? "TAUT " : "SLACK",
                            dragline.dragTension() / 1000.0f),
                 dragline.dragRopeTaut() ? RAYWHITE : ORANGE);
            line(TextFormat("chains  drag L %s R %s", chainState(Dragline::DragL), chainState(Dragline::DragR)));
        }
        line(TextFormat("bucket  %.1f t   clearance %6.2f m", cfg::kBucketMass / 1000.0f, dragline.bucketClearance()));
        line(TextFormat("chains  %s", modelName));

        DrawText("[A/D] swing  [W/S] hoist/lower  [Q/E] drag in/out  [F] free fall  [SPACE] brake  [R] reset  [P] pause  "
                 "[RMB] orbit  "
                 "[wheel] zoom",
                 10, GetScreenHeight() - 28, 18, BLACK);
        if (paused) DrawText("PAUSED", GetScreenWidth() - 110, 10, 20, YELLOW);
        DrawFPS(GetScreenWidth() - 100, GetScreenHeight() - 30);
        EndDrawing();
    }

    CloseWindow();
    dragline.shutdown();
    return 0;
}
