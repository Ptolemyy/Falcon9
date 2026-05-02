#define NOMINMAX
#include "GFOLD_solver_backend_api.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

extern "C" {
#include "cpg_workspace.h"
#include "cpg_solve.h"
}

namespace {

constexpr double kPi = 3.14159265358979323846;

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

void apply_initial_conditions(const GFOLDBackendConfig& cfg) {
    cpg_update_r0_2d(0, cfg.r0[0]);
    cpg_update_r0_2d(1, cfg.r0[1]);
    cpg_update_v0_2d(0, cfg.v0[0]);
    cpg_update_v0_2d(1, cfg.v0[1]);
    cpg_update_log_m0_2d(std::log(cfg.m0));

    const double glide_rad = cfg.glide_slope_deg * kPi / 180.0;
    const double cot_glide = (std::isfinite(cfg.cot_y_gs) && cfg.cot_y_gs > 0.0)
        ? cfg.cot_y_gs
        : (1.0 / std::tan(std::max(1e-6, glide_rad)));
    cpg_update_cot_glide_2d(cot_glide);
    cpg_update_cos_theta_2d(std::cos(cfg.max_angle_deg * kPi / 180.0));
}

void update_state(const GFOLDBackendConfig& cfg) {
    const double dt = cfg.tf / std::max(1.0, static_cast<double>(cfg.steps - 1));
    const double fuel_consumption = 1.0 / std::max(1e-9, cfg.g0 * cfg.Isp);

    cpg_update_dt2d(dt);
    cpg_update_g2d_dt(0, -cfg.g0 * dt);
    cpg_update_g2d_dt(1, 0.0);
    cpg_update_alpha2d_dt(fuel_consumption * dt);

    for (int i = 0; i < cfg.steps; ++i) {
        double mi = cfg.m0 - fuel_consumption * dt * cfg.T_max * cfg.throttle_max * i;
        if (mi <= 1.0) mi = 1.0;

        const double z0 = std::log(mi);
        const double c_z0_exp = std::exp(-z0);
        const double mu1 = 1.0 / (cfg.throttle_min * cfg.T_max * c_z0_exp);
        const double mu2 = 1.0 / (cfg.throttle_max * cfg.T_max * c_z0_exp);

        cpg_update_z0_2d(i, z0);
        cpg_update_mu1_2d(i, mu1);
        cpg_update_mu2_2d(i, mu2);
    }
}

} // namespace

extern "C" bool gfold_backend_solve_2d_n50(const GFOLDBackendConfig* cfg, GFOLDBackendOutput* out) {
    if (!cfg || !out) return false;
    if (cfg->steps != 50) return false;
    if (cfg->tf <= 0.0 || cfg->m0 <= 1.0 || cfg->T_max <= 0.0) return false;
    if (cfg->throttle_min <= 0.0 || cfg->throttle_max <= 0.0 || cfg->throttle_min > cfg->throttle_max) return false;
    if (out->capacity < static_cast<std::size_t>(cfg->steps)) return false;
    if (!out->t || !out->ux || !out->uy || !out->uz ||
        !out->vx || !out->vy || !out->vz ||
        !out->rx || !out->ry || !out->rz || !out->z) {
        return false;
    }

    const int compiled_steps = static_cast<int>(sizeof(cpg_sigma2d) / sizeof(cpg_sigma2d[0]));
    if (compiled_steps != cfg->steps) return false;

    apply_initial_conditions(*cfg);
    update_state(*cfg);
    cpg_solve();

    out->steps = cfg->steps;
    fill_info(*out);
    fill_limits(*out);

    if (!CPG_Result.prim) return false;

    const int steps = cfg->steps;
    double* uh = CPG_Result.prim->u2d;
    double* us = CPG_Result.prim->u2d + steps;
    double* vh = CPG_Result.prim->v2d;
    double* vs = CPG_Result.prim->v2d + steps;
    double* rh = CPG_Result.prim->r2d;
    double* rs = CPG_Result.prim->r2d + steps;
    double* z = CPG_Result.prim->z2d;

    if (!uh || !us || !vh || !vs || !rh || !rs || !z) return false;

    const double dt = cfg->tf / std::max(1.0, static_cast<double>(steps - 1));
    for (int i = 0; i < steps; ++i) {
        const std::size_t idx = static_cast<std::size_t>(i);
        out->t[idx] = cfg->elapsed_time + dt * static_cast<double>(i);
        out->ux[idx] = uh[i];
        out->uy[idx] = us[i];
        out->uz[idx] = 0.0;
        out->vx[idx] = vh[i];
        out->vy[idx] = vs[i];
        out->vz[idx] = 0.0;
        out->rx[idx] = rh[i];
        out->ry[idx] = rs[i];
        out->rz[idx] = 0.0;
        out->z[idx] = z[i];
    }

    out->terminal_mass = std::exp(z[steps - 1]);

    const int st = out->info.status;
    return (st == 0 || st == 10);
}
