#include "RigSolver.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rig
{
namespace
{
constexpr Real kInf = std::numeric_limits<Real>::infinity();

using Triplet = Eigen::Triplet<Real>;

struct Contact
{
    int      a, b;  // b < 0: ground
    Vec3     p, q;  // world contact points on a and on b
    Vec3     n;     // from b toward a
    Real     sep;   // signed separation, < 0 = penetrating
    uint64_t key;
};

// stable id of one contact feature (sphere of a against box of b, or the ground), times 4 for its 3 rows
uint64_t featureKey(int a, int sphere, int b, int box)
{
    return ((((static_cast<uint64_t>(a) * 4096u + static_cast<uint64_t>(sphere)) * 4096u + static_cast<uint64_t>(b + 1)) *
                 256u +
             static_cast<uint64_t>(box)) *
            4u);
}

void tangents(const Vec3& n, Vec3& t1, Vec3& t2)
{
    t1 = (std::abs(n.x()) < 0.9 ? Vec3::UnitX() : Vec3::UnitY()).cross(n).normalized();
    t2 = n.cross(t1);
}
} // namespace

int Solver::addBody(const Body& body)
{
    bodies.push_back(body);
    return static_cast<int>(bodies.size()) - 1;
}

void Solver::addBall(int a, const Vec3& anchorA, int b, const Vec3& anchorB) { balls_.push_back({a, b, anchorA, anchorB}); }

int Solver::addRope(int a, const Vec3& anchorA, int b, const Vec3& anchorB)
{
    Rope r;
    r.a  = a;
    r.pa = anchorA;
    r.b  = b;
    r.pb = anchorB;
    ropes_.push_back(r);
    return static_cast<int>(ropes_.size()) - 1;
}

void Solver::setRopeState(int rope, Real length, Real lengthRate, Real stiffness, Real damping)
{
    Rope& r  = ropes_[rope];
    r.length = length;
    r.rate   = lengthRate;
    r.k      = stiffness;
    r.d      = damping;
}

bool Solver::jointed(int a, int b) const
{
    for (const Ball& j : balls_)
        if ((j.a == a && j.b == b) || (j.a == b && j.b == a)) return true;
    return false;
}

void Solver::pointRow(Row& row, int a, const Vec3& p, int b, const Vec3& q, const Vec3& n) const
{
    row.body[0] = a;
    row.J[0] << n.transpose(), (p - bodies[a].x).cross(n).transpose();
    row.body[1] = b;
    if (b >= 0) row.J[1] << -n.transpose(), -(q - bodies[b].x).cross(n).transpose();
}

Real Solver::rowVelocity(const Row& row) const
{
    Real s = 0.0;
    for (int k = 0; k < 2; ++k)
    {
        if (row.body[k] < 0) continue;
        const Body& b = bodies[row.body[k]];
        s += row.J[k].head<3>().dot(b.v) + row.J[k].tail<3>().dot(b.w);
    }
    return s;
}

void Solver::buildContacts(Real h)
{
    const Params& P = params;
    std::vector<Contact> found, pair;
    auto keepDeepest = [&]() {
        std::sort(pair.begin(), pair.end(), [](const Contact& x, const Contact& y) { return x.sep < y.sep; });
        if (static_cast<int>(pair.size()) > P.maxContactsPerPair) pair.resize(P.maxContactsPerPair);
        found.insert(found.end(), pair.begin(), pair.end());
        pair.clear();
    };

    std::vector<Vec3> centres;
    const int         n = static_cast<int>(bodies.size());
    for (int a = 0; a < n; ++a)
    {
        const Body& A = bodies[a];
        if (A.spheres.empty() || A.kind == BodyKind::Kinematic) continue;
        const Real r     = A.sphereRadius;
        Real       reach = 0.0;
        centres.resize(A.spheres.size());
        for (size_t i = 0; i < A.spheres.size(); ++i)
        {
            centres[i] = A.x + A.q * A.spheres[i];
            reach      = std::max(reach, A.spheres[i].norm());
        }
        reach += r;

        if (A.collideWith & P.groundGroup)
        {
            for (size_t i = 0; i < centres.size(); ++i)
            {
                const Vec3& c   = centres[i];
                const Real  sep = c.y() - r;
                if (sep < P.margin)
                    pair.push_back({a, -1, c - Vec3(0.0, r, 0.0), Vec3(c.x(), 0.0, c.z()), Vec3::UnitY(), sep,
                                    featureKey(a, static_cast<int>(i), -1, 0)});
            }
            keepDeepest();
        }

        for (int b = 0; b < n; ++b)
        {
            const Body& B = bodies[b];
            if (b == a || B.boxes.empty() || !(A.group & B.collideWith) || !(B.group & A.collideWith) || jointed(a, b))
                continue;
            Real reachB = 0.0;
            for (const Box& box : B.boxes) reachB = std::max(reachB, box.center.norm() + box.half.norm());
            if ((A.x - B.x).norm() > reach + reachB + P.margin) continue;

            for (size_t k = 0; k < B.boxes.size(); ++k)
            {
                const Box& box = B.boxes[k];
                const Quat qb  = B.q * box.q;
                const Vec3 cb  = B.x + B.q * box.center;
                for (size_t i = 0; i < centres.size(); ++i)
                {
                    const Vec3 p  = qb.conjugate() * (centres[i] - cb); // box-local sphere centre
                    const Vec3 cl = p.cwiseMax(-box.half).cwiseMin(box.half);
                    Vec3       nl, ql;
                    Real       sep;
                    if ((p - cl).squaredNorm() > 1e-14)
                    {
                        const Vec3 d    = p - cl;
                        const Real dist = d.norm();
                        nl              = d / dist;
                        sep             = dist - r;
                        ql              = cl;
                    }
                    else // centre inside the box: out through the nearest face
                    {
                        int axis = 0;
                        (box.half - p.cwiseAbs()).minCoeff(&axis);
                        const Real s = p[axis] >= 0.0 ? 1.0 : -1.0;
                        nl           = Vec3::Zero();
                        nl[axis]     = s;
                        sep          = -(box.half[axis] - std::abs(p[axis])) - r;
                        ql           = p;
                        ql[axis]     = s * box.half[axis];
                    }
                    if (sep >= P.margin) continue;
                    const Vec3 nw = qb * nl;
                    pair.push_back({a, b, centres[i] - nw * r, cb + qb * ql, nw, sep,
                                    featureKey(a, static_cast<int>(i), b, static_cast<int>(k))});
                }
            }
            keepDeepest();
        }
    }

    // one normal + two friction rows per contact
    for (const Contact& c : found)
    {
        const int base = static_cast<int>(rowsC_.size());
        auto      warm = [&](uint64_t key) {
            const auto it = warm_.find(key);
            return it == warm_.end() ? 0.0 : it->second;
        };

        Row nr;
        pointRow(nr, c.a, c.p, c.b, c.q, c.n);
        const Real bias = c.sep > 0.0 ? c.sep / h : P.contactBeta * std::min(c.sep + P.slop, 0.0) / h;
        nr.sigma  = P.eps / (h * h);
        nr.rhs    = -(rowVelocity(nr) + bias);
        nr.lo     = 0.0;
        nr.hi     = kInf;
        nr.key    = c.key;
        nr.lambda = warm(nr.key);
        rowsC_.push_back(nr);

        Vec3 t[2];
        tangents(c.n, t[0], t[1]);
        for (int k = 0; k < 2; ++k)
        {
            Row tr;
            pointRow(tr, c.a, c.p, c.b, c.q, t[k]);
            tr.sigma  = P.eps / (h * h);
            tr.rhs    = -rowVelocity(tr);
            tr.findex = base;
            tr.hi     = P.friction; // times |normal impulse| in the LCP
            tr.key    = c.key + 1 + static_cast<uint64_t>(k);
            tr.lambda = warm(tr.key);
            rowsC_.push_back(tr);
        }
    }
    contactCount_ = static_cast<int>(found.size());
}

bool Solver::step(Real h)
{
    const Params& P = params;
    const int     n = static_cast<int>(bodies.size());

    col_.assign(n, -1);
    nCols_ = 0;
    for (int i = 0; i < n; ++i)
        if (bodies[i].kind != BodyKind::Kinematic)
        {
            col_[i] = nCols_;
            nCols_ += 6;
        }

    // 1. free velocities (a proxy arrives with its own prediction)
    for (Body& b : bodies)
    {
        if (b.kind != BodyKind::Dynamic) continue;
        b.v += h * P.gravity;
        b.v *= std::max(0.0, 1.0 - h * b.linDamping);
        b.w *= std::max(0.0, 1.0 - h * b.angDamping);
    }
    // link-to-link friction: implicit rotational damper per joint, f = -d (w_a - w_b), as a couple.
    // It needs no exactness, so a couple of Gauss-Seidel sweeps here instead of LDLT rows.
    // Proxy and kinematic bodies count as immovable (their velocity is not ours to change).
    if (P.jointDamping > 0.0)
    {
        auto invI = [&](int i) {
            return bodies[i].kind == BodyKind::Dynamic ? bodies[i].invInertiaWorld() : Mat3::Zero().eval();
        };
        std::vector<Mat3> iI(bodies.size());
        for (size_t i = 0; i < bodies.size(); ++i) iI[i] = invI(static_cast<int>(i));
        const Mat3 c = Mat3::Identity() / (h * P.jointDamping);
        for (int sweep = 0; sweep < 2; ++sweep)
            for (const Ball& j : balls_)
            {
                Body&      a = bodies[j.a];
                Body&      b = bodies[j.b];
                const Vec3 L = -(c + iI[j.a] + iI[j.b]).inverse() * (a.w - b.w);
                a.w += iI[j.a] * L;
                b.w -= iI[j.b] * L;
            }
    }

    // 2. bilateral rows: ball joints, 3 world-axis rows each, rigid
    rowsB_.clear();
    for (const Ball& j : balls_)
    {
        const Vec3 pa  = worldPoint(j.a, j.pa);
        const Vec3 pb  = worldPoint(j.b, j.pb);
        const Vec3 phi = pa - pb;
        for (int k = 0; k < 3; ++k)
        {
            Row r;
            pointRow(r, j.a, pa, j.b, pb, Vec3::Unit(k));
            r.sigma = P.eps / (h * h);
            r.rhs   = -(rowVelocity(r) + P.beta * phi[k] / h);
            rowsB_.push_back(r);
        }
    }

    // 3. unilateral rows: the ropes (pull only, implicit spring-damper), then contacts
    rowsC_.clear();
    std::vector<int> ropeRows(ropes_.size(), -1);
    for (size_t i = 0; i < ropes_.size(); ++i)
    {
        const Rope& rope = ropes_[i];
        const Vec3  pa   = worldPoint(rope.a, rope.pa);
        const Vec3  pb   = worldPoint(rope.b, rope.pb);
        const Vec3  d    = pa - pb;
        const Real  l    = d.norm();
        if (l <= 1e-9) continue;
        Row r;
        pointRow(r, rope.a, pa, rope.b, pb, d / l);
        const Real den = h * rope.k + rope.d;
        r.sigma        = 1.0 / (h * den);
        r.rhs          = -(rowVelocity(r) - rope.rate + rope.k * (l - rope.length) / den);
        r.lo           = -kInf;
        r.hi           = 0.0;
        r.lambda       = rope.lambda;
        ropeRows[i]    = static_cast<int>(rowsC_.size());
        rowsC_.push_back(r);
    }
    const int nRopeRows = static_cast<int>(rowsC_.size());
    buildContacts(h);

    // 4. assemble and solve
    const int mB = static_cast<int>(rowsB_.size());
    const int mC = static_cast<int>(rowsC_.size());
    auto      jacobian = [&](const std::vector<Row>& rows) {
        std::vector<Triplet> t;
        t.reserve(rows.size() * 12);
        for (size_t i = 0; i < rows.size(); ++i)
            for (int k = 0; k < 2; ++k)
            {
                const int b = rows[i].body[k];
                if (b < 0 || col_[b] < 0) continue;
                for (int j = 0; j < 6; ++j) t.emplace_back(static_cast<int>(i), col_[b] + j, rows[i].J[k](j));
            }
        SpMat J(static_cast<int>(rows.size()), nCols_);
        J.setFromTriplets(t.begin(), t.end());
        return J;
    };
    const SpMat JB = jacobian(rowsB_);
    const SpMat JC = jacobian(rowsC_);

    std::vector<Triplet> wt;
    for (int i = 0; i < n; ++i)
    {
        if (col_[i] < 0) continue;
        const int  c  = col_[i];
        const Mat3 iI = bodies[i].invInertiaWorld();
        for (int k = 0; k < 3; ++k) wt.emplace_back(c + k, c + k, bodies[i].invMass);
        for (int r = 0; r < 3; ++r)
            for (int k = 0; k < 3; ++k) wt.emplace_back(c + 3 + r, c + 3 + k, iI(r, k));
    }
    SpMat W(nCols_, nCols_);
    W.setFromTriplets(wt.begin(), wt.end());

    const SpMat WJBt = W * JB.transpose();
    SpMat       A    = JB * WJBt;
    for (int i = 0; i < mB; ++i) A.coeffRef(i, i) += rowsB_[i].sigma;
    if (A.rows() != patternRows_ || A.nonZeros() != patternNonZeros_) // joints are fixed: analyze once
    {
        ldlt_.analyzePattern(A);
        patternRows_     = A.rows();
        patternNonZeros_ = A.nonZeros();
    }
    ldlt_.factorize(A);
    if (ldlt_.info() != Eigen::Success) return false;

    Eigen::VectorXd rhsB(mB);
    for (int i = 0; i < mB; ++i) rhsB[i] = rowsB_[i].rhs;
    Eigen::VectorXd lamB = ldlt_.solve(rhsB);
    Eigen::VectorXd lamC = Eigen::VectorXd::Zero(mC);

    SpMat WJCt;
    if (mC > 0)
    {
        // Schur complement onto the unilateral rows: S lamC = s + w, then lamB = y - Y lamC
        // (A_BC stays sparse: a contact touches at most two bodies, so A_BC^T Y is cheap)
        WJCt                  = W * JC.transpose();
        const SpMat     ABC   = JB * WJCt;
        const SpMat     ABCt  = ABC.transpose();
        Eigen::MatrixXd S     = Eigen::MatrixXd(JC * WJCt);
        for (int i = 0; i < mC; ++i) S(i, i) += rowsC_[i].sigma;
        const Eigen::MatrixXd Y = ldlt_.solve(Eigen::MatrixXd(ABC));
        S.noalias() -= ABCt * Y;
        Eigen::VectorXd s(mC);
        for (int i = 0; i < mC; ++i) s[i] = rowsC_[i].rhs;
        s -= ABCt * lamB;

        // projected Gauss-Seidel, warm started; friction bounded by mu |normal impulse|
        for (int i = 0; i < mC; ++i) lamC[i] = rowsC_[i].lambda;
        for (int it = 0; it < P.pgsIterations; ++it)
        {
            Real maxDelta = 0.0;
            for (int i = 0; i < mC; ++i)
            {
                const Row& r  = rowsC_[i];
                Real       lo = r.lo, hi = r.hi;
                if (r.findex >= 0)
                {
                    hi = r.hi * std::abs(lamC[r.findex]);
                    lo = -hi;
                }
                const Real res = s[i] - S.col(i).dot(lamC); // S is symmetric
                const Real x   = std::clamp(lamC[i] + res / S(i, i), lo, hi);
                maxDelta       = std::max(maxDelta, std::abs(x - lamC[i]));
                lamC[i]        = x;
            }
            if (maxDelta < P.pgsTolerance) break;
        }
        lamB.noalias() -= Y * lamC;
    }

    // 5. velocity update u+ = u* + W J^T lambda, then integrate the dynamic bodies
    impulse_ = JB.transpose() * lamB;
    if (mC > 0) impulse_ += JC.transpose() * lamC;
    const Eigen::VectorXd du = W * impulse_;
    for (int i = 0; i < n; ++i)
    {
        if (col_[i] < 0) continue;
        bodies[i].v += du.segment<3>(col_[i]);
        bodies[i].w += du.segment<3>(col_[i] + 3);
    }
    for (Body& b : bodies)
    {
        if (b.kind != BodyKind::Dynamic) continue;
        b.x += h * b.v;
        Quat dq(0.0, b.w.x(), b.w.y(), b.w.z());
        dq.coeffs() = 0.5 * h * (dq * b.q).coeffs();
        b.q.coeffs() += dq.coeffs();
        b.q.normalize();
    }

    // 6. keep results: rope impulses, contact impulses for the next warm start
    for (size_t i = 0; i < ropes_.size(); ++i) ropes_[i].lambda = ropeRows[i] >= 0 ? lamC[ropeRows[i]] : 0.0;
    warm_.clear();
    for (int i = nRopeRows; i < mC; ++i) warm_[rowsC_[i].key] = lamC[i];
    return true;
}

void Solver::bodyImpulse(int body, Vec3& linear, Vec3& angular) const
{
    linear = angular = Vec3::Zero();
    if (body < 0 || body >= static_cast<int>(col_.size()) || col_[body] < 0 || impulse_.size() == 0) return;
    linear  = impulse_.segment<3>(col_[body]);
    angular = impulse_.segment<3>(col_[body] + 3);
}
} // namespace rig

