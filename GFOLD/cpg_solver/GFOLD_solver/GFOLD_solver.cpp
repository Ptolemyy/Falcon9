#include "GFOLD_solver.hpp"
#include "GFOLD_solver_backend_api.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

extern "C" bool gfold_backend_solve_n3(const GFOLDBackendConfig* cfg, GFOLDBackendOutput* out);
extern "C" bool gfold_backend_solve_n10(const GFOLDBackendConfig* cfg, GFOLDBackendOutput* out);
extern "C" bool gfold_backend_solve_n25(const GFOLDBackendConfig* cfg, GFOLDBackendOutput* out);
extern "C" bool gfold_backend_solve_n50(const GFOLDBackendConfig* cfg, GFOLDBackendOutput* out);
extern "C" bool gfold_backend_solve_n100(const GFOLDBackendConfig* cfg, GFOLDBackendOutput* out);

namespace {

constexpr double kPi = 3.14159265358979323846;

using BackendSolveFn = bool (*)(const GFOLDBackendConfig*, GFOLDBackendOutput*);

BackendSolveFn resolve_backend(int solver_n) {
    switch (solver_n) {
        case 3: return &gfold_backend_solve_n3;     // p3 backend
        case 10: return &gfold_backend_solve_n10;
        case 25: return &gfold_backend_solve_n25;
        case 50: return &gfold_backend_solve_n50;
        case 100: return &gfold_backend_solve_n100;
        default: return nullptr;
    }
}

GFOLDBackendConfig to_backend_cfg(const GFOLDConfig& cfg) {
    GFOLDBackendConfig out;
    out.steps = cfg.steps;
    out.tf = cfg.tf;
    out.elapsed_time = cfg.elapsed_time;
    out.g0 = cfg.g0;
    out.Isp = cfg.Isp;
    out.T_max = cfg.T_max;
    out.throttle_min = cfg.throttle_min;
    out.throttle_max = cfg.throttle_max;
    out.m0 = cfg.m0;
    out.r0[0] = cfg.r0[0];
    out.r0[1] = cfg.r0[1];
    out.r0[2] = cfg.r0[2];
    out.v0[0] = cfg.v0[0];
    out.v0[1] = cfg.v0[1];
    out.v0[2] = cfg.v0[2];
    out.glide_slope_deg = cfg.glide_slope_deg;
    out.cot_y_gs = cfg.cot_y_gs;
    out.max_angle_deg = cfg.max_angle_deg;
    return out;
}

} // namespace

GFOLDSolver::GFOLDSolver(const GFOLDConfig& cfg) : cfg_(cfg) {}

void GFOLDSolver::set_config(const GFOLDConfig& cfg) {
    cfg_ = cfg;
}

bool GFOLDSolver::solve(int solver_n) {
    cfg_.solver_n = solver_n;
    return solve_with_backend_n(solver_n);
}

bool GFOLDSolver::solve() {
    return solve_with_backend_n(cfg_.solver_n);
}

bool GFOLDSolver::solve_with_backend_n(int solver_n) {
    const BackendSolveFn backend = resolve_backend(solver_n);
    if (!backend) {
        last_solution_ = GFOLDSolution{};
        last_info_ = GFOLDSolverInfo{};
        last_limits_ = GFOLDSolverLimits{};
        last_info_.status = -901; // unknown backend n
        last_terminal_mass_ = std::numeric_limits<double>::quiet_NaN();
        return false;
    }

    if (cfg_.steps <= 0) {
        last_solution_ = GFOLDSolution{};
        last_info_ = GFOLDSolverInfo{};
        last_limits_ = GFOLDSolverLimits{};
        last_info_.status = -902; // invalid steps
        last_terminal_mass_ = std::numeric_limits<double>::quiet_NaN();
        return false;
    }

    const std::size_t cap = static_cast<std::size_t>(cfg_.steps);
    last_solution_ = GFOLDSolution{};
    last_solution_.steps = cfg_.steps;
    last_solution_.t.assign(cap, 0.0);
    last_solution_.ux.assign(cap, 0.0);
    last_solution_.uy.assign(cap, 0.0);
    last_solution_.uz.assign(cap, 0.0);
    last_solution_.vx.assign(cap, 0.0);
    last_solution_.vy.assign(cap, 0.0);
    last_solution_.vz.assign(cap, 0.0);
    last_solution_.rx.assign(cap, 0.0);
    last_solution_.ry.assign(cap, 0.0);
    last_solution_.rz.assign(cap, 0.0);
    last_solution_.z.assign(cap, 0.0);

    GFOLDBackendConfig backend_cfg = to_backend_cfg(cfg_);
    GFOLDBackendOutput backend_out;
    backend_out.capacity = cap;
    backend_out.t = last_solution_.t.data();
    backend_out.ux = last_solution_.ux.data();
    backend_out.uy = last_solution_.uy.data();
    backend_out.uz = last_solution_.uz.data();
    backend_out.vx = last_solution_.vx.data();
    backend_out.vy = last_solution_.vy.data();
    backend_out.vz = last_solution_.vz.data();
    backend_out.rx = last_solution_.rx.data();
    backend_out.ry = last_solution_.ry.data();
    backend_out.rz = last_solution_.rz.data();
    backend_out.z = last_solution_.z.data();

    const bool ok = backend(&backend_cfg, &backend_out);
    last_info_.status = backend_out.info.status;
    last_info_.iter = backend_out.info.iter;
    last_info_.obj_val = backend_out.info.obj_val;
    last_info_.pri_res = backend_out.info.pri_res;
    last_info_.dua_res = backend_out.info.dua_res;

    last_limits_.feastol = backend_out.limits.feastol;
    last_limits_.abstol = backend_out.limits.abstol;
    last_limits_.reltol = backend_out.limits.reltol;
    last_limits_.feastol_inacc = backend_out.limits.feastol_inacc;
    last_limits_.abstol_inacc = backend_out.limits.abstol_inacc;
    last_limits_.reltol_inacc = backend_out.limits.reltol_inacc;
    last_limits_.maxit = backend_out.limits.maxit;
    last_terminal_mass_ = backend_out.terminal_mass;

    if (!ok) {
        if (last_info_.status == -999) last_info_.status = -903;
        return false;
    }
    return true;
}

int GFOLDSolver::status() const {
    return last_info_.status;
}

GFOLDSolverInfo GFOLDSolver::info() const {
    return last_info_;
}

GFOLDSolverLimits GFOLDSolver::limits() const {
    return last_limits_;
}

GFOLDSolution GFOLDSolver::solution() const {
    return last_solution_;
}

double GFOLDSolver::terminal_mass() const {
    return last_terminal_mass_;
}

GFOLDThrustProfile GFOLDSolver::compute_thrust_profile() const {
    GFOLDThrustProfile prof;
    const int steps = last_solution_.steps;
    if (steps <= 0) return prof;
    if (static_cast<int>(last_solution_.ux.size()) != steps ||
        static_cast<int>(last_solution_.uy.size()) != steps ||
        static_cast<int>(last_solution_.uz.size()) != steps ||
        static_cast<int>(last_solution_.z.size()) != steps) {
        return prof;
    }

    prof.t.reserve(static_cast<std::size_t>(steps));
    prof.thrust_N.reserve(static_cast<std::size_t>(steps));
    prof.angle_deg.reserve(static_cast<std::size_t>(steps));

    const double RAD2DEG = 180.0 / kPi;
    for (int i = 0; i < steps; ++i) {
        const double ux = last_solution_.ux[static_cast<std::size_t>(i)];
        const double uy = last_solution_.uy[static_cast<std::size_t>(i)];
        const double uz = last_solution_.uz[static_cast<std::size_t>(i)];
        const double m = std::exp(last_solution_.z[static_cast<std::size_t>(i)]);
        const double norm_u = std::sqrt(ux * ux + uy * uy + uz * uz);

        prof.t.push_back(last_solution_.t.empty() ? (cfg_.tf / steps) * i : last_solution_.t[static_cast<std::size_t>(i)]);
        prof.thrust_N.push_back(norm_u * m);

        double cosv = 1.0;
        if (norm_u > 0.0) {
            cosv = ux / norm_u;
            cosv = std::clamp(cosv, -1.0, 1.0);
        }
        prof.angle_deg.push_back(std::acos(cosv) * RAD2DEG);
    }
    return prof;
}
