#include "planner_upfg.hpp"

#include <cmath>

namespace falcon9 {

namespace {

Vec3 constrained_position_component(const Vec3& v, const Vec3& r_hat, const Vec3& n_hat) {
    return r_hat * dot3(v, r_hat) + n_hat * dot3(v, n_hat);
}

struct ThrustIntegrals {
    double j = 0.0;   // Integral of thrust acceleration.
    double s = 0.0;   // Integral of tau * thrust acceleration.
    double q = 0.0;   // Integral of tau * (tgo - tau) * thrust acceleration.
    double h = 0.0;   // Integral of (tgo - tau) * thrust acceleration.
};

ThrustIntegrals thrust_integrals(
    double thrust_N,
    double isp_s,
    double throttle,
    double mass_kg,
    double tgo_s) {
    ThrustIntegrals out;
    const double tgo = std::max(0.0, tgo_s);
    const double throttle_cmd = clampd(throttle, 0.0, 1.0);
    if (thrust_N <= 0.0 || isp_s <= 0.0 || mass_kg <= 1.0 || tgo <= 0.0 || throttle_cmd <= 0.0) {
        return out;
    }

    const double ve = std::max(1e-6, isp_s * kG0);
    const double mdot = thrust_N * throttle_cmd / ve;
    constexpr int kSamples = 32;
    const double d_tau = tgo / static_cast<double>(kSamples);
    for (int i = 0; i < kSamples; ++i) {
        const double tau = (static_cast<double>(i) + 0.5) * d_tau;
        const double mass = std::max(1.0, mass_kg - mdot * tau);
        const double accel = thrust_N * throttle_cmd / mass;
        out.j += accel * d_tau;
        out.s += tau * accel * d_tau;
        out.q += tau * (tgo - tau) * accel * d_tau;
    }
    out.h = tgo * out.j - out.s;
    return out;
}

double linear_steering_dv_required(
    const Vec3& lambda0,
    const Vec3& lambda1,
    double thrust_N,
    double isp_s,
    double throttle,
    double mass_kg,
    double tgo_s) {
    const double tgo = std::max(0.0, tgo_s);
    const double throttle_cmd = clampd(throttle, 0.0, 1.0);
    if (thrust_N <= 0.0 || isp_s <= 0.0 || mass_kg <= 1.0 || tgo <= 0.0 || throttle_cmd <= 0.0) {
        return 0.0;
    }

    const double ve = std::max(1e-6, isp_s * kG0);
    const double mdot = thrust_N * throttle_cmd / ve;
    constexpr int kSamples = 32;
    const double d_tau = tgo / static_cast<double>(kSamples);
    double out = 0.0;
    for (int i = 0; i < kSamples; ++i) {
        const double tau = (static_cast<double>(i) + 0.5) * d_tau;
        const double mass = std::max(1.0, mass_kg - mdot * tau);
        const double accel = thrust_N * throttle_cmd / mass;
        out += accel * norm3(lambda0 + lambda1 * tau) * d_tau;
    }
    return out;
}

struct UpfgLinearLaw {
    Vec3 lambda0;
    Vec3 lambda1;
    Vec3 vgo;
    Vec3 pos_residual;
    double available_dv = 0.0;
    double required_dv = 0.0;
};

UpfgLinearLaw solve_powered_explicit_guidance_law(
    const StateVector3D& state,
    const Vec3& target_r,
    const Vec3& target_v,
    const Vec3& r_hat,
    const Vec3& n_hat,
    double thrust_N,
    double isp_s,
    double throttle,
    double tgo_s) {
    UpfgLinearLaw out;
    const double r = std::max(1.0, norm3(state.r_m));
    const Vec3 gravity = state.r_m * (-kMu / (r * r * r));
    const ThrustIntegrals integrals = thrust_integrals(
        thrust_N,
        isp_s,
        throttle,
        state.m_kg,
        tgo_s);
    out.available_dv = integrals.j;
    out.vgo = target_v - state.v_mps - gravity * tgo_s;
    const Vec3 rgo =
        target_r -
        state.r_m -
        state.v_mps * tgo_s -
        gravity * (0.5 * tgo_s * tgo_s);

    Vec3 lambda1;
    if (integrals.j > 1e-9) {
        const double denom = integrals.q - integrals.h * integrals.s / integrals.j;
        out.pos_residual = constrained_position_component(
            rgo - out.vgo * (integrals.h / integrals.j),
            r_hat,
            n_hat);
        if (std::abs(denom) > 1e-9) {
            lambda1 = out.pos_residual / denom;
        }
        out.lambda0 = (out.vgo - lambda1 * integrals.s) / integrals.j;
    }
    out.lambda1 = lambda1;
    out.required_dv = linear_steering_dv_required(
        out.lambda0,
        out.lambda1,
        thrust_N,
        isp_s,
        throttle,
        state.m_kg,
        tgo_s);
    return out;
}

Vec3 clamp_direction_gamma(const Vec3& dir, const Vec3& r_hat, double gamma_min_rad, double gamma_max_rad) {
    const double radial = dot3(dir, r_hat);
    Vec3 horizontal = dir - r_hat * radial;
    double horizontal_norm = norm3(horizontal);
    if (horizontal_norm <= 1e-9) {
        horizontal = reject3({1.0, 0.0, 0.0}, r_hat);
        if (norm3(horizontal) <= 1e-9) horizontal = reject3({0.0, 1.0, 0.0}, r_hat);
        horizontal = normalize3(horizontal);
        horizontal_norm = 1.0;
    } else {
        horizontal = horizontal / horizontal_norm;
    }
    const double gamma = std::atan2(radial, std::max(1e-9, horizontal_norm));
    const double clamped = clampd(gamma, gamma_min_rad, gamma_max_rad);
    return normalize3(horizontal * std::cos(clamped) + r_hat * std::sin(clamped));
}

}  // namespace

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

void propagate_state3d_coast(StateVector3D& s, double dt) {
    if (!s.valid) return;
    const double r = std::max(1.0, norm3(s.r_m));
    const Vec3 gravity = s.r_m * (-kMu / (r * r * r));
    s.v_mps += gravity * dt;
    s.r_m += s.v_mps * dt;
    const double rn = norm3(s.r_m);
    if (rn < kRe) {
        const Vec3 rhat = normalize3(s.r_m);
        s.r_m = rhat * kRe;
        const double vr = dot3(s.v_mps, rhat);
        if (vr < 0.0) s.v_mps -= rhat * vr;
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

UpfgCommand upfg_compute_command_3d(
    const StateVector3D& state,
    const UpfgVehicle& vehicle,
    const UpfgTarget3D& target,
    const UpfgSettings& settings,
    double max_tgo_s,
    double dt) {
    UpfgCommand out;
    if (!state.valid) return out;

    const double thrust = std::max(1.0, vehicle.thrust_N);
    const double max_throttle = clampd(vehicle.max_throttle, 0.0, 1.0);
    const double min_throttle = clampd(vehicle.min_throttle, 0.0, max_throttle);

    const Vec3 n_hat = normalize3(target.plane_normal_eci);
    Vec3 r_hat = normalize3(reject3(state.r_m, n_hat));
    if (dot3(r_hat, r_hat) < 1e-10) r_hat = normalize3(state.r_m);
    Vec3 t_hat = normalize3(cross3(n_hat, r_hat));
    if (dot3(t_hat, state.v_mps) < 0.0) t_hat *= -1.0;

    const Vec3 target_r = r_hat * target.target_r_m;
    const Vec3 target_v = r_hat * target.target_vr_mps + t_hat * target.target_vt_mps;

    const Vec3 pos_err = constrained_position_component(target_r - state.r_m, r_hat, n_hat);
    const double max_tgo = std::max(settings.terminal_time_s, max_tgo_s);
    const double lo_tgo = std::max(settings.terminal_time_s, 0.5);
    const double hi_tgo = std::max(lo_tgo, max_tgo);
    UpfgLinearLaw law_hi = solve_powered_explicit_guidance_law(
        state,
        target_r,
        target_v,
        r_hat,
        n_hat,
        thrust,
        vehicle.isp_s,
        max_throttle,
        hi_tgo);
    double lo = lo_tgo;
    double hi = hi_tgo;
    bool bracketed = law_hi.available_dv >= law_hi.required_dv;
    UpfgLinearLaw law = law_hi;
    if (bracketed) {
        for (int i = 0; i < 18; ++i) {
            const double mid = 0.5 * (lo + hi);
            UpfgLinearLaw mid_law = solve_powered_explicit_guidance_law(
                state,
                target_r,
                target_v,
                r_hat,
                n_hat,
                thrust,
                vehicle.isp_s,
                max_throttle,
                mid);
            if (mid_law.available_dv >= mid_law.required_dv) {
                hi = mid;
                law = mid_law;
            } else {
                lo = mid;
            }
        }
        out.tgo_s = hi;
    } else {
        out.tgo_s = hi_tgo;
    }

    law = solve_powered_explicit_guidance_law(
        state,
        target_r,
        target_v,
        r_hat,
        n_hat,
        thrust,
        vehicle.isp_s,
        max_throttle,
        out.tgo_s);
    out.lambda0_eci = law.lambda0;
    out.lambda1_eci = law.lambda1;
    out.vgo_mps = norm3(law.vgo);

    const double steering_tau = clampd(0.5 * std::max(0.0, dt), 0.0, out.tgo_s);
    const Vec3 steering_vec = law.lambda0 + law.lambda1 * steering_tau;
    const double steering_norm = norm3(steering_vec);
    out.thrust_dir_eci = steering_norm > 1e-9 ? steering_vec / steering_norm : t_hat;
    out.thrust_dir_eci = clamp_direction_gamma(
        out.thrust_dir_eci,
        r_hat,
        deg2rad(settings.gamma_min_deg),
        deg2rad(settings.gamma_max_deg));

    const double radial_dir = dot3(out.thrust_dir_eci, r_hat);
    Vec3 horizontal_dir = out.thrust_dir_eci - r_hat * radial_dir;
    const double tangential_dir = std::max(1e-9, norm3(horizontal_dir));
    out.gamma_cmd_rad = std::atan2(radial_dir, tangential_dir);

    out.throttle = max_throttle;
    if (max_throttle > min_throttle && out.tgo_s <= settings.terminal_time_s + std::max(0.0, dt) && law.available_dv > 1e-9) {
        const double terminal_fraction = clampd(law.required_dv / law.available_dv, 0.0, 1.0);
        out.throttle = clampd(max_throttle * terminal_fraction, min_throttle, max_throttle);
    }

    const double dr = norm3(pos_err);
    out.converged =
        dr <= 1500.0 &&
        out.vgo_mps <= 22.0 &&
        out.tgo_s <= std::max(settings.terminal_time_s + 1.0, 9.0);
    return out;
}

}  // namespace falcon9
