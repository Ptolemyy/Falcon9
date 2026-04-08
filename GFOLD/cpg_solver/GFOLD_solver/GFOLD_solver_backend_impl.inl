#include "GFOLD_solver_backend_api.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

extern "C" {
#include "cpg_workspace.h"
#include "cpg_solve.h"
}

#ifndef GFOLD_BACKEND_FN
#error "GFOLD_BACKEND_FN must be defined before including GFOLD_solver_backend_impl.inl"
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;

void apply_initial_conditions(const GFOLDBackendConfig& cfg) {
    cpg_update_r0(0, cfg.r0[0]);
    cpg_update_r0(1, cfg.r0[1]);
    cpg_update_r0(2, cfg.r0[2]);

    cpg_update_v0(0, cfg.v0[0]);
    cpg_update_v0(1, cfg.v0[1]);
    cpg_update_v0(2, cfg.v0[2]);

    cpg_update_log_m0(std::log(cfg.m0));
    const double glide_rad = cfg.glide_slope_deg * kPi / 180.0;
    const bool has_cot_override = std::isfinite(cfg.cot_y_gs) && (cfg.cot_y_gs > 0.0);
#if defined(GFOLD_BACKEND_GLIDE_PARAM_COT)
    const double cot_y_gs = has_cot_override ? cfg.cot_y_gs : (1.0 / std::tan(glide_rad));
    cpg_update_cot_y_gs(cot_y_gs);
#else
    const double sin_y_gs = has_cot_override
        ? (1.0 / std::sqrt(1.0 + cfg.cot_y_gs * cfg.cot_y_gs))
        : std::sin(glide_rad);
    cpg_update_sin_y_gs(sin_y_gs);
#endif
    cpg_update_cos_theta_deg(std::cos(cfg.max_angle_deg * kPi / 180.0));
}

void update_state(const GFOLDBackendConfig& cfg) {
    const double dt = cfg.tf / static_cast<double>(cfg.steps);
    const double fuel_consumption = 1.0 / (cfg.g0 * cfg.Isp);

    cpg_update_dt(dt);
    cpg_update_a_dt(fuel_consumption * dt);
    cpg_update_g_dt(0, -cfg.g0 * dt);

    for (int i = 0; i < cfg.steps; ++i) {
        double mi = cfg.m0 - fuel_consumption * dt * cfg.T_max * cfg.throttle_max * i;
        if (mi <= 1.0) mi = 1.0;

        const double z0 = std::log(mi);
        const double c_z0_exp = std::exp(-z0);
        const double mu_1 = 1.0 / (cfg.throttle_min * cfg.T_max * c_z0_exp);
        const double mu_2 = 1.0 / (cfg.throttle_max * cfg.T_max * c_z0_exp);

        cpg_update_z0(i, z0);
        cpg_update_mu_1(i, mu_1);
        cpg_update_mu_2(i, mu_2);
    }
}

void fill_info(GFOLDBackendOutput& out) {
    if (!CPG_Result.info) return;
    out.info.status = CPG_Result.info->status;
    out.info.iter = CPG_Result.info->iter;
    out.info.obj_val = CPG_Result.info->obj_val;
    out.info.pri_res = CPG_Result.info->pri_res;
    out.info.dua_res = CPG_Result.info->dua_res;
}

void fill_limits(GFOLDBackendOutput& out) {
    out.limits.feastol = Canon_Settings.feastol;
    out.limits.abstol = Canon_Settings.abstol;
    out.limits.reltol = Canon_Settings.reltol;
    out.limits.feastol_inacc = Canon_Settings.feastol_inacc;
    out.limits.abstol_inacc = Canon_Settings.abstol_inacc;
    out.limits.reltol_inacc = Canon_Settings.reltol_inacc;
    out.limits.maxit = Canon_Settings.maxit;
}

} // namespace

extern "C" bool GFOLD_BACKEND_FN(const GFOLDBackendConfig* cfg, GFOLDBackendOutput* out) {
    if (!cfg || !out) return false;
    if (cfg->steps <= 0) return false;

    const int compiled_steps = static_cast<int>(sizeof(cpg_s) / sizeof(cpg_s[0]));
    if (cfg->steps != compiled_steps) return false;

    if (out->capacity < static_cast<std::size_t>(cfg->steps)) return false;
    if (!out->t || !out->ux || !out->uy || !out->uz ||
        !out->vx || !out->vy || !out->vz ||
        !out->rx || !out->ry || !out->rz || !out->z) {
        return false;
    }

    apply_initial_conditions(*cfg);
    update_state(*cfg);
    cpg_solve();

    out->steps = cfg->steps;
    fill_info(*out);
    fill_limits(*out);

    if (!CPG_Result.prim) return false;

    const int steps = cfg->steps;
    double* ux = CPG_Result.prim->u;
    double* uy = CPG_Result.prim->u + steps;
    double* uz = CPG_Result.prim->u + 2 * steps;
    double* vx = CPG_Result.prim->v;
    double* vy = CPG_Result.prim->v + steps;
    double* vz = CPG_Result.prim->v + 2 * steps;
    double* rx = CPG_Result.prim->r;
    double* ry = CPG_Result.prim->r + steps;
    double* rz = CPG_Result.prim->r + 2 * steps;
    double* z = CPG_Result.prim->z;

    if (!ux || !uy || !uz || !vx || !vy || !vz || !rx || !ry || !rz || !z) return false;

    const double dt = cfg->tf / static_cast<double>(steps);
    for (int i = 0; i < steps; ++i) {
        const std::size_t idx = static_cast<std::size_t>(i);
        out->t[idx] = cfg->elapsed_time + dt * static_cast<double>(i);
        out->ux[idx] = ux[i];
        out->uy[idx] = uy[i];
        out->uz[idx] = uz[i];
        out->vx[idx] = vx[i];
        out->vy[idx] = vy[i];
        out->vz[idx] = vz[i];
        out->rx[idx] = rx[i];
        out->ry[idx] = ry[i];
        out->rz[idx] = rz[i];
        out->z[idx] = z[i];
    }

    out->terminal_mass = std::exp(z[steps - 1]);

    const int st = out->info.status;
    return (st == 0 || st == 10);
}
