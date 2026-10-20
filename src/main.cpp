#include "Dragline.h"
#include "Recording.h"
#include "Renderer.h"

#include <raylib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <string>

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

// Horizontal slider, dragged with the left mouse button. Returns true while it is being dragged.
bool slider(int id, Rectangle track, const char* label, float& value, float lo, float hi)
{
    static int      grabbed = -1; // a press on a track grabs that slider until the button is released
    const Vector2   mouse   = GetMousePosition();
    const Rectangle hit{track.x - 8.0f, track.y - 8.0f, track.width + 16.0f, track.height + 16.0f};
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouse, hit)) grabbed = id;
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) grabbed = -1;
    const bool dragging = grabbed == id;
    if (dragging) value = lo + std::clamp((mouse.x - track.x) / track.width, 0.0f, 1.0f) * (hi - lo);

    const float t    = (value - lo) / (hi - lo);
    const float knob = track.x + t * track.width;
    DrawText(label, static_cast<int>(track.x) + 1, static_cast<int>(track.y) - 21, 18, BLACK);
    DrawText(label, static_cast<int>(track.x), static_cast<int>(track.y) - 22, 18, RAYWHITE);
    DrawRectangleRec(track, Color{40, 40, 40, 200});
    DrawRectangleRec(Rectangle{track.x, track.y, knob - track.x, track.height}, Color{240, 200, 80, 255});
    DrawRectangle(static_cast<int>(knob) - 5, static_cast<int>(track.y) - 5, 10, static_cast<int>(track.height) + 10,
                  dragging ? YELLOW : RAYWHITE);
    return dragging;
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
    // sliders: max drum pulls and swing speed
    float       dragPullKN  = dragline.dragDrum().params().motorForce / 1000.0f;
    float       hoistPullKN = dragline.drum().params().motorForce / 1000.0f;
    float       swingRpm    = dragline.swingMaxRate() * 60.0f / (2.0f * PI);
    // input recording since the last reset; [F9] or the REC button saves it (replay: dragline_selftest replay <file>)
    rec::Recording recording;
    recording.model = model;
    recording.dt    = kFixedDt;
    std::string savedMsg;
    float       savedMsgTime = 0.0f;

    while (!WindowShouldClose())
    {
        // --- input ---
        if (IsKeyPressed(KEY_R))
        {
            dragline.reset();
            accumulator = 0.0f;
            freeFall    = false;
            recording.steps.clear();
        }
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
                recording.steps.push_back({controls, swingRpm, hoistPullKN, dragPullKN});
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
            line(TextFormat("drag    payout %6.2f m   line speed %+5.2f m/s   %s   tension %6.1f kN   motor %6.1f kN",
                            dragDrum.payout(), dragDrum.speed(), dragline.dragRopeTaut() ? "TAUT " : "SLACK",
                            dragline.dragTension() / 1000.0f, -dragDrum.motorForce() / 1000.0f),
                 dragline.dragRopeTaut() ? RAYWHITE : ORANGE);
            line(TextFormat("chains  drag L %s R %s", chainState(Dragline::DragL), chainState(Dragline::DragR)));
        }
        line(TextFormat("bucket  %.1f t   clearance %6.2f m", cfg::kBucketMass / 1000.0f, dragline.bucketClearance()));
        line(TextFormat("chains  %s", modelName));

        DrawText("[A/D] swing  [W/S] hoist/lower  [Q/E] drag in/out  [F] free fall  [SPACE] brake  [R] reset  [P] pause  "
                 "[RMB] orbit  "
                 "[wheel] zoom",
                 10, GetScreenHeight() - 28, 18, BLACK);
        // live tuning sliders, top right
        const float sliderX = static_cast<float>(GetScreenWidth()) - 340.0f;
        if (slider(0, {sliderX, 60.0f, 300.0f, 10.0f}, TextFormat("swing speed max %4.2f rpm", swingRpm), swingRpm,
                   0.2f, 8.0f))
            dragline.setSwingMaxRate(swingRpm * 2.0f * PI / 60.0f);
        if (slider(1, {sliderX, 115.0f, 300.0f, 10.0f}, TextFormat("hoist pull max %5.0f kN", hoistPullKN), hoistPullKN,
                   10.0f, 250.0f))
            dragline.setHoistMotorForce(hoistPullKN * 1000.0f);
        if (dragline.hasDrag() &&
            slider(2, {sliderX, 170.0f, 300.0f, 10.0f}, TextFormat("drag pull max %5.0f kN", dragPullKN), dragPullKN,
                   10.0f, 250.0f))
            dragline.setDragMotorForce(dragPullKN * 1000.0f);

        // save the recording: button under the sliders, or F9
        const Rectangle recButton{sliderX, 200.0f, 300.0f, 32.0f};
        const bool      overRec = CheckCollisionPointRec(GetMousePosition(), recButton);
        DrawRectangleRec(recButton, overRec ? Color{200, 40, 40, 255} : Color{160, 30, 30, 230});
        DrawText(TextFormat("SAVE RECORDING [F9]  (%.0f s)", recording.steps.size() * kFixedDt),
                 static_cast<int>(recButton.x) + 10, static_cast<int>(recButton.y) + 8, 18, RAYWHITE);
        if (IsKeyPressed(KEY_F9) || (overRec && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)))
        {
            std::filesystem::create_directories("recordings");
            const std::time_t now = std::time(nullptr);
            std::tm           tm{};
            localtime_s(&tm, &now);
            char name[64];
            std::strftime(name, sizeof(name), "recordings/rec_%Y%m%d_%H%M%S.txt", &tm);
            savedMsg     = rec::save(name, recording, dragline)
                             ? std::string("saved ") + std::filesystem::absolute(name).string()
                             : std::string("could not write ") + name;
            savedMsgTime = static_cast<float>(GetTime());
        }
        if (!savedMsg.empty() && GetTime() - savedMsgTime < 6.0)
            DrawText(savedMsg.c_str(), 10, GetScreenHeight() - 56, 18, YELLOW);
        if (paused) DrawText("PAUSED", GetScreenWidth() - 110, 10, 20, YELLOW);
        DrawFPS(GetScreenWidth() - 100, GetScreenHeight() - 30);
        EndDrawing();
    }

    CloseWindow();
    dragline.shutdown();
    return 0;
}
