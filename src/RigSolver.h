#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <cstdint>
#include <unordered_map>
#include <vector>

// Direct rigging solver (dragline_chain_solver_spec.md, option A). Knows nothing about PhysX.
// Maximal coordinates, velocity level, double precision. Per step:
//   free velocities -> ball-joint rows solved exactly (sparse LDLT) -> rope + contacts as a small
//   boxed LCP on the Schur complement (projected Gauss-Seidel) -> velocity update -> integrate.
// A Proxy body (the PhysX bucket) takes part in the solve with its real mass, but is not integrated:
// the caller reads the impulse the rig put on it and hands that to PhysX.
namespace rig
{
using Real = double;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

enum class BodyKind
{
    Dynamic,   // integrated here
    Proxy,     // state (with its free velocity already predicted) set by the caller every step
    Kinematic, // prescribed motion, no DOFs (boom tip / house)
};

struct Box
{
    Vec3 center = Vec3::Zero(); // in the body (COM) frame
    Quat q      = Quat::Identity();
    Vec3 half   = Vec3::Zero();
};

struct Body
{
    BodyKind kind = BodyKind::Dynamic;
    Vec3     x    = Vec3::Zero(); // centre of mass
    Quat     q    = Quat::Identity();
    Vec3     v    = Vec3::Zero();
    Vec3     w    = Vec3::Zero();
    Real     invMass    = 0.0;
    Vec3     invInertia = Vec3::Zero(); // principal axes = body frame
    Real     linDamping = 0.0;
    Real     angDamping = 0.0;

    // contacts: spheres (body frame centres, one radius) hit boxes of other bodies and the ground
    uint32_t          group       = 0;
    uint32_t          collideWith = 0;
    std::vector<Vec3> spheres;
    Real              sphereRadius = 0.0;
    std::vector<Box>  boxes;

    Mat3 invInertiaWorld() const
    {
        const Mat3 R = q.toRotationMatrix();
        return R * invInertia.asDiagonal() * R.transpose();
    }
    Vec3 pointVelocity(const Vec3& worldPoint) const { return v + w.cross(worldPoint - x); }
};

struct Params
{
    Vec3     gravity{0.0, -9.81, 0.0};
    Real     beta          = 0.3;   // Baumgarte factor of rigid rows
    Real     jointDamping  = 0.0;   // N m s/rad, relative rotation at every ball joint (link-to-link friction)
    Real     eps           = 1e-9;  // rigid-row regularization (x 1/h^2)
    Real     contactBeta   = 0.3;
    Real     slop          = 0.003; // m of penetration left uncorrected
    Real     margin        = 0.08;  // m, speculative contact distance
    Real     friction      = 0.5;
    int      pgsIterations = 80;
    Real     pgsTolerance  = 1e-7;  // N s, stop when no impulse changes more than this
    int      maxContactsPerPair = 2; // deepest per (body, other body / ground)
    uint32_t groundGroup   = 1;     // ground plane y = 0
};

class Solver
{
public:
    Params            params;
    std::vector<Body> bodies;

    int  addBody(const Body& body);
    void addBall(int a, const Vec3& anchorA, int b, const Vec3& anchorB); // anchors in body frames
    int  addRope(int a, const Vec3& anchorA, int b, const Vec3& anchorB); // returns the rope index
    // paid-out length, its rate (rope can only pull), axial stiffness and damping
    void setRopeState(int rope, Real length, Real lengthRate, Real stiffness, Real damping);
    void clearWarmStart() { warm_.clear(); }

    bool step(Real h); // false if the factorization failed (free velocities applied, nothing else)

    // results of the last step
    void bodyImpulse(int body, Vec3& linear, Vec3& angular) const; // what the rig applied to it (about its COM)
    Real ropeImpulse(int rope) const { return ropes_[rope].lambda; } // <= 0: tension = -impulse / h
    int  contactCount() const { return contactCount_; }
    Vec3 worldPoint(int body, const Vec3& local) const { return bodies[body].x + bodies[body].q * local; }

private:
    using Row6 = Eigen::Matrix<Real, 1, 6>;
    using SpMat = Eigen::SparseMatrix<Real>;

    struct Ball
    {
        int  a, b;
        Vec3 pa, pb;
    };
    struct Rope
    {
        int  a = -1, b = -1;
        Vec3 pa = Vec3::Zero(), pb = Vec3::Zero();
        Real length = 1.0, rate = 0.0, k = 1.0, d = 0.0;
        Real lambda = 0.0; // last step's impulse, also its warm start
    };
    struct Row
    {
        int      body[2] = {-1, -1};
        Row6     J[2];
        Real     rhs    = 0.0; // -(J u* - target + bias)
        Real     sigma  = 0.0; // compliance on the diagonal
        Real     lo     = 0.0, hi = 0.0;
        int      findex = -1;  // friction row: index of its normal row (within the same block)
        uint64_t key    = 0;   // contact feature id for warm starting
        Real     lambda = 0.0;
    };

    // row along n through world point p on a and q on b (b < 0: static world): rel. velocity (a - b) . n
    void pointRow(Row& row, int a, const Vec3& p, int b, const Vec3& q, const Vec3& n) const;
    Real rowVelocity(const Row& row) const; // J u at the current (free) velocities, kinematic bodies included
    void buildContacts(Real h);
    bool jointed(int a, int b) const;

    std::vector<Ball> balls_;
    std::vector<Rope> ropes_; // hoist, drag, ...: their rows come first in rowsC_
    std::vector<int>  col_; // body -> first column of its DOFs, -1 if kinematic
    int               nCols_ = 0;

    std::vector<Row> rowsB_; // bilateral
    std::vector<Row> rowsC_; // rope + contacts (unilateral, LCP)
    Eigen::SimplicialLDLT<SpMat, Eigen::Lower, Eigen::AMDOrdering<int>> ldlt_;
    Eigen::Index patternRows_ = -1, patternNonZeros_ = -1;
    Eigen::VectorXd impulse_; // J^T lambda of the last step, per column
    std::unordered_map<uint64_t, Real> warm_;
    int contactCount_ = 0;
};
} // namespace rig
