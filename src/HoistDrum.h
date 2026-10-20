#pragma once

#include <algorithm>
#include <cmath>

// One-degree-of-freedom hoist drum model, expressed in rope coordinates.
// speed > 0 = paying out (lowering), speed < 0 = reeling in (hoisting).
// The drum knows nothing about PhysX: the caller passes in the load the rope
// carries and applies the resulting payout to the rope joint.
struct HoistDrumParams
{
    float drumMass      = 800.0f;    // kg, drum + gearbox inertia reflected to the rope (J / r^2)
    float motorForce    = 90000.0f;  // N, max motor line pull
    float motorGain     = 40000.0f;  // N per m/s of speed error
    float maxSpeed      = 2.0f;      // m/s, line speed at full lever
    float maxAccel      = 1.0f;      // m/s^2, commanded line speed ramp (real winches take 1-2 s to full speed)
    float stopSpeed     = 0.02f;     // m/s, lever neutral: the holding brake sets once the drum is this slow
    float friction      = 1500.0f;   // N, bearing / gear friction, always present
    float holdBrake     = 80000.0f;  // N, spring-applied holding brake (powered mode, lever neutral)
    float freeFallBrake = 80000.0f;  // N, operator brake at full pedal (free-fall mode)
    float minPayout     = 2.0f;      // m, two-block limit
    float maxPayout     = 45.0f;     // m, rope left on the drum
};

struct HoistDrumInput
{
    float lever    = 0.0f;  // -1 = hoist .. +1 = lower (powered mode only)
    bool  freeFall = false; // clutch released: motor disconnected, gravity drives payout
    float brake    = 0.0f;  // 0..1 operator brake pedal (free-fall mode only)
};

class HoistDrum
{
public:
    HoistDrum(const HoistDrumParams& params, float payout) : p_(params), payout_(payout) {}

    // loadForce: force the load puts on the rope (N, + pulls rope off the drum).
    // loadMass:  mass rigidly coupled to the drum through a taut rope (0 when slack).
    void step(const HoistDrumInput& in, float loadForce, float loadMass, float dt)
    {
        float drive  = loadForce;
        float resist = p_.friction;
        motorOut_    = 0.0f;
        holding_     = false;

        if (in.freeFall)
        {
            resist += std::clamp(in.brake, 0.0f, 1.0f) * p_.freeFallBrake;
        }
        else if (std::fabs(in.lever) < 1e-3f && vRef_ == 0.0f && std::fabs(speed_) < p_.stopSpeed)
        {
            holding_ = true;
            resist += p_.holdBrake;
        }
        else
        {
            // speed servo with load feed-forward, tracking a ramped reference: lever moves and lever
            // release both change line speed at maxAccel instead of jerking the rigging
            const float vCmd = std::fabs(in.lever) < 1e-3f ? 0.0f : std::clamp(in.lever, -1.0f, 1.0f) * p_.maxSpeed;
            const float dv   = p_.maxAccel * dt;
            vRef_            = std::clamp(vCmd, vRef_ - dv, vRef_ + dv);
            motorOut_ = std::clamp(p_.motorGain * (vRef_ - speed_) - loadForce, -p_.motorForce, p_.motorForce);
            drive += motorOut_;
        }

        // Coulomb friction / brake: sticks until the drive exceeds it, never reverses motion
        if (speed_ != 0.0f || std::fabs(drive) > resist)
        {
            const float dir = speed_ != 0.0f ? sign(speed_) : sign(drive);
            const float a   = (drive - resist * dir) / (p_.drumMass + loadMass);
            const float v   = speed_ + a * dt;
            speed_          = (speed_ != 0.0f && v * speed_ < 0.0f) ? 0.0f : v;
        }

        payout_ += speed_ * dt;
        if (payout_ <= p_.minPayout || payout_ >= p_.maxPayout)
        {
            payout_ = std::clamp(payout_, p_.minPayout, p_.maxPayout);
            speed_  = 0.0f;
            vRef_   = 0.0f; // no wind-up against the limit
        }
        if (in.freeFall) vRef_ = speed_; // re-engaging the motor starts from the current speed
    }

    void reset(float payout)
    {
        payout_ = payout;
        speed_  = 0.0f;
        vRef_   = 0.0f;
    }

    float payout() const { return payout_; }
    float speed() const { return speed_; }
    float motorForce() const { return motorOut_; }
    bool  holdingBrake() const { return holding_; }
    const HoistDrumParams& params() const { return p_; }
    void setMotorForce(float force) { p_.motorForce = std::max(force, 0.0f); } // live tuning (max line pull)

private:
    static float sign(float x) { return x < 0.0f ? -1.0f : 1.0f; }

    HoistDrumParams p_;
    float payout_   = 0.0f;
    float speed_    = 0.0f;
    float vRef_     = 0.0f; // ramped speed command
    float motorOut_ = 0.0f;
    bool  holding_  = false;
};
