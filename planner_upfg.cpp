#include "planner_upfg.hpp"

#include <cmath>

namespace falcon9 {

void propagate_polar_coast(PolarState& s, double dt) {
    const double ar = (s.vt * s.vt) / std::max(1.0, s.r) - kMu / std::max(1.0, s.r * s.r);
    const double at = -(s.vr * s.vt) / std::max(1.0, s.r);
    s.vr += ar * dt;
    s.vt += at * dt;
    s.r += s.vr * dt;
    s.theta += (s.vt / std::max(1.0, s.r)) * dt;
    if (s.r < kRe) {
        s.r = kRe;
        if (s.vr < 0.0) s.vr = 0.0;
    }
}

UpfgCommand upfg_compute_command(
    const PolarState& state,
    const UpfgVehicle& vehicle,
    const UpfgTarget& target,
    const UpfgSettings& settings,
    double prev_tgo_s,
    double dt) {
    UpfgCommand out;

    const double thrust = std::max(1.0, vehicle.thrust_N);
    const double max_throttle = clampd(vehicle.max_throttle, 0.0, 1.0);
    const double min_throttle = clampd(vehicle.min_throttle, 0.0, max_throttle);
    const double acc_max = (thrust * std::max(0.05, max_throttle)) / std::max(1.0, state.m);

    const double dv_r = target.target_vr_mps - state.vr;
    const double dv_t = target.target_vt_mps - state.vt;
    const double dr = target.target_r_m - state.r;
    const double dr_abs = std::abs(dr);
    out.vgo_mps = std::sqrt(dv_r * dv_r + dv_t * dv_t) + 0.004 * dr_abs;

    const double tgo_pos = std::sqrt(2.0 * dr_abs / std::max(0.5, 0.55 * acc_max));

    const double tgo_raw = std::max(
        settings.terminal_time_s,
        std::max(
            out.vgo_mps / std::max(0.5, 0.78 * acc_max),
            0.85 * tgo_pos));

    if (prev_tgo_s > 0.0) {
        out.tgo_s = std::max(settings.terminal_time_s, std::min(prev_tgo_s - 0.35 * dt, tgo_raw));
    } else {
        out.tgo_s = tgo_raw;
    }

    const double tgo = std::max(settings.terminal_time_s, out.tgo_s);
    const double radial_pos_term =
        settings.radial_position_gain * 2.0 * (dr - state.vr * tgo) / std::max(1.0, tgo * tgo);
    const double radial_vel_term = settings.radial_gain * dv_r / tgo;
    const double tangential_vel_term = settings.tangential_gain * dv_t / tgo;

    const double ar_fb = clampd(
        radial_pos_term + radial_vel_term,
        -settings.max_radial_accel,
        settings.max_radial_accel);
    const double at_fb = clampd(
        tangential_vel_term,
        -settings.max_tangential_accel,
        settings.max_tangential_accel);

    double thrust_r_cmd = ar_fb - ((state.vt * state.vt) / std::max(1.0, state.r) - kMu / std::max(1.0, state.r * state.r));
    double thrust_t_cmd = at_fb + (state.vr * state.vt) / std::max(1.0, state.r);
    if (thrust_t_cmd < 1e-5) thrust_t_cmd = 1e-5;

    double gamma_raw = std::atan2(thrust_r_cmd, thrust_t_cmd);
    gamma_raw = clampd(gamma_raw, deg2rad(settings.gamma_min_deg), deg2rad(settings.gamma_max_deg));
    out.gamma_cmd_rad = gamma_raw;

    const double thrust_acc_cmd = std::sqrt(thrust_r_cmd * thrust_r_cmd + thrust_t_cmd * thrust_t_cmd);
    const double throttle_raw = thrust_acc_cmd / std::max(1e-6, thrust / std::max(1.0, state.m));
    out.throttle = clampd(throttle_raw, min_throttle, max_throttle);

    const double speed_err = std::hypot(dv_r, dv_t);
    out.converged =
        std::abs(dr) <= 1200.0 &&
        speed_err <= 18.0 &&
        out.tgo_s <= std::max(settings.terminal_time_s + 1.0, 9.0);
    return out;
}

}  // namespace falcon9
