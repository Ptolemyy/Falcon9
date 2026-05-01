#include "planner_mission.hpp"

#include "planner_recovery.hpp"
#include "planner_upfg.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>

namespace falcon9 {

namespace {

struct Stage1Target {
    double sep_time_s = 0.0;
    double sep_alt_km = 0.0;
    double sep_speed_mps = 0.0;
    double sep_gamma_deg = 0.0;
    // When true the ascent burn may consume the full propellant load (ignoring
    // the landing reserve).  This mirrors the previous behaviour and is needed
    // for the "force stage-1 burnout" mission mode where recovery is disabled
    // on purpose.
    bool allow_full_burn = false;
};

std::wstring fnum(double v, int p) {
    std::wostringstream oss;
    oss << std::fixed << std::setprecision(p) << v;
    return oss.str();
}

std::wstring ftime(double s) {
    int secs = static_cast<int>(std::lround(std::max(0.0, s)));
    const int mm = secs / 60;
    const int ss = secs % 60;
    std::wostringstream oss;
    oss << L"T+" << std::setfill(L'0') << std::setw(2) << mm << L":" << std::setw(2) << ss;
    return oss.str();
}

double local_circular_speed(double alt_km) {
    return std::sqrt(kMu / std::max(1.0, kRe + alt_km * 1000.0));
}

double stage2_orbit_penalty(
    const OrbitTarget& target,
    const Stage2Result& stage2,
    double& rp_err_km,
    double& ra_err_km,
    double& target_r_err_km,
    double& fpa_err_deg,
    bool& orbit_ok) {
    rp_err_km = stage2.orbit.rp_km - target.rp_km;
    ra_err_km = stage2.orbit.ra_km - target.ra_km;
    target_r_err_km = (stage2.seco.r - target.r_target_m) / 1000.0;
    fpa_err_deg = stage2.orbit.flight_path_deg - target.fpa_target_deg;

    double penalty = 0.0;
    penalty += std::max(0.0, std::abs(rp_err_km) - 10.0) * 20.0;
    penalty += std::max(0.0, std::abs(ra_err_km) - 16.0) * 12.0;
    penalty += std::max(0.0, std::abs(fpa_err_deg) - 1.0) * 32.0;
    penalty += std::max(0.0, std::abs(target_r_err_km) - 8.0) * 8.0;
    if (stage2.orbit.rp_km < 80.0) penalty += (80.0 - stage2.orbit.rp_km) * 45.0;

    orbit_ok =
        std::abs(rp_err_km) <= 40.0 &&
        std::abs(ra_err_km) <= 60.0 &&
        std::abs(target_r_err_km) <= 40.0 &&
        stage2.orbit.rp_km >= 80.0;
    return penalty;
}

Stage1Result simulate_stage1_candidate(
    const MissionRequest& request,
    const Stage1Target& target) {
    Stage1Result out;
    struct FlatState {
        double x = 0.0;
        double z = 0.0;
        double vx = 0.0;
        double vz = 0.0;
        double m = 0.0;
    };

    FlatState s;
    {
        const double lat = deg2rad(request.lat_deg);
        const double inc = deg2rad(std::abs(request.incl_deg));
        const double c_lat = std::cos(lat);
        double sin_az = 0.0;
        if (std::abs(c_lat) > 1e-6) sin_az = clampd(std::cos(inc) / c_lat, -1.0, 1.0);
        const double vrot = kOmega * kRe * c_lat;
        s.vx = vrot * std::max(0.0, sin_az);
    }
    s.m = request.s1_dry_kg + request.s1_prop_kg + request.s2_dry_kg + request.s2_prop_kg + request.payload_kg;

    const double sep_delay_s = clampd(request.s1_sep_delay_s, 1.5, 6.0);
    const double burn_s = std::max(90.0, target.sep_time_s - sep_delay_s);
    const double thrust = request.s1_thrust_kN * 1000.0;
    const double mdot_nom = thrust / std::max(1e-6, request.s1_isp_s * kG0);
    // Reserve a fraction of stage-1 propellant for boost-back / reentry / landing burns.
    // The MissionRequest exposes s1_reserve as a ratio (e.g. 0.08 = 8%), historically
    // parsed from settings but never enforced inside the ascent burn.  Without this cap
    // the ascent loop happily consumed 100% of s1_prop_kg whenever the chosen separation
    // time exceeded burnout, leaving the recovery sim with rem_prop = 0 and forcing a
    // catastrophic infeasible verdict.  The cap is bypassed when the caller explicitly
    // requests a burnout sweep so the existing "no recovery" mission profiles still
    // behave the same.
    const double s1_reserve_ratio = clampd(request.s1_reserve, 0.0, 0.40);
    const double s1_max_consumable_kg = target.allow_full_burn
        ? request.s1_prop_kg
        : std::max(0.0, request.s1_prop_kg * (1.0 - s1_reserve_ratio));
    const double cda = 9.0;
    const double q_target_kpa = clampd(request.s1_target_maxq_kpa, 15.0, std::max(15.0, request.q_limit_kpa));
    const double q_target_time_s = clampd(0.49 * burn_s, 67.0, 76.0);
    const double guide_start_s = clampd(burn_s - 88.0, 30.0, burn_s - 16.0);
    const double guide_dt = 0.20;

    out.traj.reserve(320);
    out.traj.push_back({0.0, 0.0, 0.0});
    out.guide_start_s = guide_start_s;
    out.min_throttle = 1.0;

    double t = 0.0;
    double used = 0.0;
    double gamma_cmd_rad = deg2rad(89.2);
    double upfg_prev_tgo = burn_s - guide_start_s;

    const UpfgVehicle upfg_vehicle{
        thrust,
        request.s1_isp_s,
        0.25,
        1.0,
    };
    const UpfgTarget upfg_target{
        kRe + target.sep_alt_km * 1000.0,
        target.sep_speed_mps * std::sin(deg2rad(target.sep_gamma_deg)),
        target.sep_speed_mps * std::cos(deg2rad(target.sep_gamma_deg)),
    };
    const UpfgSettings upfg_settings{
        6.0,
        0.0,
        28.0,
        1.10,
        1.85,
        0.95,
        6.0,
        13.0,
    };

    while (t < burn_s - 1e-9 && used < s1_max_consumable_kg - 1e-9) {
        const double dt = std::min(guide_dt, burn_s - t);
        const double alt_m = std::max(0.0, s.z);
        const double alt_km = alt_m / 1000.0;
        const double speed = std::hypot(s.vx, s.vz);
        const double dens = rho(alt_m);
        const double q = 0.5 * dens * speed * speed / 1000.0;
        if (q > out.max_q) {
            out.max_q = q;
            out.t_max_q = t;
        }

        const double g = grav(alt_m);
        const double drag = 0.5 * dens * speed * speed * cda;
        double drag_ax = 0.0;
        double drag_az = 0.0;
        if (speed > 1e-6) {
            const double invm = 1.0 / std::max(1.0, s.m);
            drag_ax = (drag * invm) * (s.vx / speed);
            drag_az = (drag * invm) * (s.vz / speed);
        }

        double gamma_raw = deg2rad(89.2);
        double throttle_cmd = 1.0;
        if (t < guide_start_s) {
            double pitch_time_deg = 89.2;
            if (t >= 7.5) {
                if (t < 20.0) {
                    pitch_time_deg = 89.2 - 0.82 * (t - 7.5);
                } else if (t < 55.0) {
                    pitch_time_deg = 78.95 - 0.62 * (t - 20.0);
                } else if (t < 92.0) {
                    pitch_time_deg = 57.25 - 0.54 * (t - 55.0);
                } else if (t < 120.0) {
                    pitch_time_deg = 37.27 - 0.40 * (t - 92.0);
                } else {
                    pitch_time_deg = 26.07 - 0.26 * (t - 120.0);
                }
            }

            double pitch_alt_deg = 89.2;
            if (alt_km < 1.0) {
                pitch_alt_deg = 89.2;
            } else if (alt_km < 12.0) {
                pitch_alt_deg = 89.2 - 1.20 * (alt_km - 1.0);
            } else if (alt_km < 35.0) {
                pitch_alt_deg = 76.00 - 1.10 * (alt_km - 12.0);
            } else if (alt_km < 70.0) {
                pitch_alt_deg = 50.70 - 0.62 * (alt_km - 35.0);
            } else {
                pitch_alt_deg = 29.00 - 0.18 * (alt_km - 70.0);
            }
            gamma_raw = deg2rad(0.64 * pitch_time_deg + 0.36 * pitch_alt_deg);
            gamma_raw -= deg2rad(2.0) * smoothstep(35.0, 85.0, t);
            gamma_raw -= deg2rad(3.5) * smoothstep(85.0, guide_start_s, t);

            const double bucket_window =
                smoothstep(q_target_time_s - 20.0, q_target_time_s - 4.0, t) *
                (1.0 - smoothstep(q_target_time_s + 8.0, q_target_time_s + 18.0, t));
            if (bucket_window > 1e-4) {
                const double q_err = q_target_kpa - q;
                const double throttle_bucket = clampd(0.72 + 0.022 * q_err, 0.54, 1.0);
                throttle_cmd = std::min(1.0, lerpd(1.0, throttle_bucket, bucket_window));
            }
            if (alt_km > 35.0 && t < 0.86 * burn_s) {
                throttle_cmd = std::max(throttle_cmd, 0.90);
            }
            if (q < 0.80 * q_target_kpa && t > q_target_time_s + 6.0) throttle_cmd = 1.0;
        } else {
            PolarState pseudo;
            pseudo.r = kRe + std::max(0.0, s.z);
            pseudo.theta = s.x / kRe;
            pseudo.vr = s.vz;
            pseudo.vt = s.vx;
            pseudo.m = s.m;
            const UpfgCommand cmd = upfg_compute_command(pseudo, upfg_vehicle, upfg_target, upfg_settings, upfg_prev_tgo, dt);
            upfg_prev_tgo = cmd.tgo_s;
            out.tgo_final_s = cmd.tgo_s;
            out.vgo_final_mps = cmd.vgo_mps;
            gamma_raw = cmd.gamma_cmd_rad;
            throttle_cmd = clampd(cmd.throttle, 0.25, 1.0);
        }

        if (t < 5.0) gamma_raw = deg2rad(89.2);
        gamma_raw = clampd(gamma_raw, deg2rad(5.0), deg2rad(89.4));
        const double rate_limit_rad_s = deg2rad((t < 45.0) ? 1.10 : 0.90);
        const double dmax = rate_limit_rad_s * dt;
        gamma_cmd_rad += clampd(gamma_raw - gamma_cmd_rad, -dmax, dmax);
        gamma_cmd_rad = clampd(gamma_cmd_rad, deg2rad(5.0), deg2rad(89.4));

        if (throttle_cmd < out.min_throttle - 1e-9) {
            out.min_throttle = throttle_cmd;
            out.t_min_throttle = t;
        }

        const double dm = std::min(mdot_nom * throttle_cmd * dt, s1_max_consumable_kg - used);
        const double mdot = dm / std::max(1e-9, dt);
        const double thrust_now = mdot * request.s1_isp_s * kG0;
        const double invm = 1.0 / std::max(1.0, s.m);

        double ax = (thrust_now * std::cos(gamma_cmd_rad)) * invm;
        double az = (thrust_now * std::sin(gamma_cmd_rad)) * invm - g;
        if (speed > 1e-6) {
            ax -= drag_ax;
            az -= drag_az;
        }

        s.vx += ax * dt;
        s.vz += az * dt;
        s.x += s.vx * dt;
        s.z += s.vz * dt;
        s.m -= dm;
        used += dm;
        t += dt;

        if (s.z < 0.0) {
            s.z = 0.0;
            if (s.vz < 0.0) s.vz = 0.0;
        }
        if (t - out.traj.back().t >= 1.0 || t >= burn_s - 1e-6) {
            out.traj.push_back({t, s.x / 1000.0, std::max(0.0, s.z / 1000.0)});
        }
    }

    out.meco = {kRe + std::max(0.0, s.z), s.x / kRe, s.vz, s.vx, s.m};
    out.meco_s = t;
    out.burn_s = t;
    out.used_prop = used;
    out.rem_prop = std::max(0.0, request.s1_prop_kg - used);

    double coast_elapsed = 0.0;
    const double sep_delay_s_dt = 0.20;
    while (coast_elapsed < sep_delay_s - 1e-9) {
        const double dt = std::min(sep_delay_s_dt, sep_delay_s - coast_elapsed);
        const double alt_m = std::max(0.0, s.z);
        const double speed = std::hypot(s.vx, s.vz);
        const double dens = rho(alt_m);
        const double g = grav(alt_m);
        const double drag = 0.5 * dens * speed * speed * cda;
        double drag_ax = 0.0;
        double drag_az = 0.0;
        if (speed > 1e-6) {
            const double invm = 1.0 / std::max(1.0, s.m);
            drag_ax = (drag * invm) * (s.vx / speed);
            drag_az = (drag * invm) * (s.vz / speed);
        }
        double ax = 0.0;
        double az = -g;
        if (speed > 1e-6) {
            ax -= drag_ax;
            az -= drag_az;
        }
        s.vx += ax * dt;
        s.vz += az * dt;
        s.x += s.vx * dt;
        s.z += s.vz * dt;
        t += dt;
        coast_elapsed += dt;
        if (s.z < 0.0) {
            s.z = 0.0;
            if (s.vz < 0.0) s.vz = 0.0;
        }
        if (t - out.traj.back().t >= 1.0 || coast_elapsed >= sep_delay_s - 1e-6) {
            out.traj.push_back({t, s.x / 1000.0, std::max(0.0, s.z / 1000.0)});
        }
    }

    out.sep = {kRe + std::max(0.0, s.z), s.x / kRe, s.vz, s.vx, s.m};
    out.sep_s = t;
    const double sep_alt_km = (out.sep.r - kRe) / 1000.0;
    const double sep_speed_mps = std::hypot(out.sep.vr, out.sep.vt);
    const double sep_gamma_deg = rad2deg(std::atan2(out.sep.vr, std::max(1.0, out.sep.vt)));
    out.target_alt_err_km = sep_alt_km - target.sep_alt_km;
    out.target_speed_err_mps = sep_speed_mps - target.sep_speed_mps;
    out.target_gamma_err_deg = sep_gamma_deg - target.sep_gamma_deg;
    out.converged =
        std::isfinite(sep_alt_km) &&
        std::isfinite(sep_speed_mps) &&
        std::isfinite(sep_gamma_deg) &&
        std::isfinite(out.max_q);
    out.envelope_ok =
        out.max_q <= request.q_limit_kpa + 1e-6 &&
        sep_alt_km >= 45.0 &&
        sep_alt_km <= 110.0 &&
        sep_speed_mps >= 1600.0 &&
        sep_speed_mps <= 3600.0 &&
        sep_gamma_deg >= -3.0 &&
        sep_gamma_deg <= 20.0;
    return out;
}

Stage2Result simulate_stage2_candidate(
    const MissionRequest& request,
    const Stage1Result& stage1,
    const OrbitTarget& orbit_target) {
    Stage2Result out;
    const PolarState ignition_seed{
        stage1.sep.r,
        stage1.sep.theta,
        stage1.sep.vr,
        stage1.sep.vt,
        request.s2_dry_kg + request.s2_prop_kg + request.payload_kg,
    };
    const double ignition_delay = clampd(request.s2_ignition_delay_s, 3.0, 15.0);
    const double target_r = kRe + orbit_target.cutoff_alt_km * 1000.0;
    const double target_a = kRe + 0.5 * (orbit_target.rp_km + orbit_target.ra_km) * 1000.0;
    const double target_ra = kRe + orbit_target.ra_km * 1000.0;
    const double target_e = (target_ra - (kRe + orbit_target.rp_km * 1000.0)) / std::max(1.0, target_ra + (kRe + orbit_target.rp_km * 1000.0));
    const double target_p = target_a * (1.0 - target_e * target_e);
    const double target_h = std::sqrt(kMu * target_p);
    const double thrust = request.s2_thrust_kN * 1000.0;
    const double mdot_nom = thrust / std::max(1e-6, request.s2_isp_s * kG0);
    const double burn_max = request.s2_prop_kg / std::max(1e-6, mdot_nom);

    const UpfgVehicle upfg_vehicle{thrust, request.s2_isp_s, 1.0, 1.0};
    const UpfgTarget upfg_target{
        orbit_target.r_target_m,
        orbit_target.vr_target_mps,
        orbit_target.vt_target_mps,
    };
    const UpfgSettings upfg_settings{
        6.0,
        -6.0,
        20.0,
        1.0,
        1.0,
        1.0,
        8.0,
        12.0,
    };

    auto simulate_candidate = [&](double gamma_bias_deg, double climb_start_u, double climb_end_u, double vr_gain, double vt_gain, double end_gamma_deg, double rate_limit_deg_s) {
        Stage2Result cand;
        cand.traj.reserve(960);

        PolarState s = ignition_seed;
        cand.traj.push_back({stage1.sep_s, s.theta * (kRe / 1000.0), (s.r - kRe) / 1000.0});
        double t_global = stage1.sep_s;
        double t_local = 0.0;
        while (t_local < ignition_delay - 1e-9) {
            const double dt = std::min(0.5, ignition_delay - t_local);
            propagate_polar_coast(s, dt);
            t_local += dt;
            t_global += dt;
            if (t_global - cand.traj.back().t >= 1.0 || t_local >= ignition_delay - 1e-6) {
                cand.traj.push_back({t_global, s.theta * (kRe / 1000.0), std::max(0.0, (s.r - kRe) / 1000.0)});
            }
        }

        cand.ignition = s;
        cand.ignition_s = t_global;

        double used = 0.0;
        double gamma_cmd = clampd(std::atan2(s.vr, std::max(1.0, s.vt)) + deg2rad(gamma_bias_deg), deg2rad(-2.0), deg2rad(45.0));
        double burn_elapsed = 0.0;
        double upfg_prev_tgo = burn_max;
        PolarState best_state = s;
        OrbitMetrics best_orbit{};
        double best_cutoff_s = cand.ignition_s;
        double best_used = 0.0;
        double best_rp_err_km = 0.0;
        double best_ra_err_km = 0.0;
        double best_target_r_err_km = 0.0;
        double best_fpa_err_deg = 0.0;
        double best_peak_alt_km = std::max(0.0, (s.r - kRe) / 1000.0);
        double best_penalty = std::numeric_limits<double>::infinity();
        double best_selection_score = std::numeric_limits<double>::infinity();
        bool best_orbit_ok = false;
        bool best_converged = false;
        double best_tgo = burn_max;
        double best_vgo = std::numeric_limits<double>::infinity();
        size_t best_traj_size = cand.traj.size();
        const double r_ignition = s.r;
        double peak_alt_km = std::max(0.0, (s.r - kRe) / 1000.0);
        bool crashed = false;

        while (burn_elapsed < burn_max - 1e-9 && used < request.s2_prop_kg - 1e-9) {
            const double dt = std::min(0.5, burn_max - burn_elapsed);
            const double t_go = std::max(4.0, burn_max - burn_elapsed);
            const double gamma_now = std::atan2(s.vr, std::max(1.0, s.vt));
            const double climb_u = smoothstep(climb_start_u * burn_max, climb_end_u * burn_max, burn_elapsed);
            const double target_r_path = lerpd(r_ignition, target_r, climb_u);
            const double vt_target = target_h / std::max(1.0, s.r);
            const double vr_target = clampd((target_r_path - s.r) / std::max(12.0, 0.95 * t_go), -160.0, 160.0);

            const double gamma_prog = lerpd(
                clampd(gamma_now + deg2rad(0.35 * gamma_bias_deg), deg2rad(-1.0), deg2rad(10.0)),
                deg2rad(end_gamma_deg),
                smoothstep(0.10 * burn_max, 0.96 * burn_max, burn_elapsed));

            const double ar_fb =
                clampd(
                    vr_gain * (vr_target - s.vr) / t_go +
                        0.60 * 2.0 * (target_r_path - s.r - s.vr * t_go) / std::max(1.0, t_go * t_go),
                    -6.0,
                    6.0);
            const double at_fb =
                clampd(
                    vt_gain * (vt_target - s.vt) / t_go +
                        0.03 * (target_r - s.r) / std::max(1.0, t_go * t_go),
                    -3.0,
                    11.0);

            double thrust_r_cmd = ar_fb - ((s.vt * s.vt) / std::max(1.0, s.r) - kMu / std::max(1.0, s.r * s.r));
            double thrust_t_cmd = at_fb + (s.vr * s.vt) / std::max(1.0, s.r);
            if (thrust_t_cmd < 1e-4) thrust_t_cmd = 1e-4;
            const double gamma_fb = std::atan2(thrust_r_cmd, thrust_t_cmd);
            double gamma_raw = lerpd(gamma_prog, gamma_fb, smoothstep(0.12 * burn_max, 0.84 * burn_max, burn_elapsed));
            const double alt_to_go = std::max(0.0, target_r - s.r);
            const double alt_now_km = std::max(0.0, (s.r - kRe) / 1000.0);
            if (alt_to_go > 1000.0 && burn_elapsed < 0.94 * burn_max) {
                const double thrust_acc_nom = thrust / std::max(1.0, s.m);
                const double grav_deficit = kMu / std::max(1.0, s.r * s.r) - (s.vt * s.vt) / std::max(1.0, s.r);
                const double climb_acc = clampd((alt_to_go - 10000.0) / 25000.0, 0.0, 2.5);
                const double radial_need = std::max(0.0, grav_deficit + climb_acc);
                if (thrust_acc_nom > 1e-6) {
                    const double sin_floor = clampd(radial_need / thrust_acc_nom, 0.0, 0.98);
                    gamma_raw = std::max(gamma_raw, std::asin(sin_floor));
                }
                if (s.vr < -20.0) {
                    gamma_raw = std::max(gamma_raw, deg2rad(25.0));
                }
            }
            if (burn_elapsed > 0.70 * burn_max) {
                gamma_raw = lerpd(gamma_raw, deg2rad(end_gamma_deg), smoothstep(0.70 * burn_max, burn_max, burn_elapsed));
            }
            if (s.r > target_r - 50000.0) {
                gamma_raw = lerpd(gamma_raw, deg2rad(end_gamma_deg), smoothstep(target_r - 50000.0, target_r + 5000.0, s.r));
            }
            if (s.r > target_r - 35000.0) {
                gamma_raw = lerpd(gamma_raw, deg2rad(orbit_target.fpa_target_deg), smoothstep(target_r - 35000.0, target_r + 2000.0, s.r));
            }
            const double gamma_hi_deg =
                (alt_to_go > 60000.0) ? 75.0 :
                (alt_to_go > 25000.0) ? 60.0 :
                (alt_to_go > 8000.0) ? 35.0 : 22.0;
            gamma_raw = clampd(gamma_raw, deg2rad(-5.0), deg2rad(gamma_hi_deg));
            const double rate_limit_deg =
                (alt_to_go > 60000.0) ? std::max(rate_limit_deg_s, 5.0) :
                (alt_to_go > 25000.0) ? std::max(rate_limit_deg_s, 3.5) :
                std::max(rate_limit_deg_s, 1.5);
            const double rate_limit = deg2rad(rate_limit_deg) * dt;
            gamma_cmd += clampd(gamma_raw - gamma_cmd, -rate_limit, rate_limit);

            const UpfgCommand cmd = upfg_compute_command(s, upfg_vehicle, upfg_target, upfg_settings, upfg_prev_tgo, dt);
            upfg_prev_tgo = cmd.tgo_s;

            const double dm = std::min(mdot_nom * dt, request.s2_prop_kg - used);
            const double mdot = dm / std::max(1e-9, dt);
            const double thrust_now = mdot * request.s2_isp_s * kG0;
            const double invm = 1.0 / std::max(1.0, s.m);

            const double ar = (thrust_now * std::sin(gamma_cmd)) * invm +
                              (s.vt * s.vt) / std::max(1.0, s.r) -
                              kMu / std::max(1.0, s.r * s.r);
            const double at = (thrust_now * std::cos(gamma_cmd)) * invm -
                              (s.vr * s.vt) / std::max(1.0, s.r);

            s.vr += ar * dt;
            s.vt += at * dt;
            s.r += s.vr * dt;
            s.theta += (s.vt / std::max(1.0, s.r)) * dt;
            s.m -= dm;
            used += dm;
            burn_elapsed += dt;
            t_global += dt;
            if (s.r < kRe) {
                crashed = true;
                break;
            }

            if (t_global - cand.traj.back().t >= 1.0 || burn_elapsed >= burn_max - 1e-6) {
                cand.traj.push_back({t_global, s.theta * (kRe / 1000.0), std::max(0.0, (s.r - kRe) / 1000.0)});
            }
            peak_alt_km = std::max(peak_alt_km, std::max(0.0, (s.r - kRe) / 1000.0));

            if (burn_elapsed >= 120.0 && (s.r >= target_r - 3000.0 || burn_elapsed >= 0.80 * burn_max)) {
                const OrbitMetrics orbit = orbit_metrics_from_state(s);
                double rp_err_km = 0.0;
                double ra_err_km = 0.0;
                double target_r_err_km = 0.0;
                double fpa_err_deg = 0.0;
                bool orbit_ok = false;
                const Stage2Result probe_state{
                    {},
                    s,
                    orbit,
                    cand.ignition_s,
                    burn_elapsed,
                    t_global,
                    used,
                    std::max(0.0, request.s2_prop_kg - used),
                };
                const double penalty = stage2_orbit_penalty(orbit_target, probe_state, rp_err_km, ra_err_km, target_r_err_km, fpa_err_deg, orbit_ok);
                double selection_score = penalty;
                if (!orbit_ok) {
                    const double rem_prop = std::max(0.0, request.s2_prop_kg - used);
                    const double speed_shortfall = std::max(0.0, orbit_target.speed_target_mps - orbit.speed_mps);
                    const double vt_shortfall = std::max(0.0, vt_target - s.vt);
                    selection_score += std::max(0.0, speed_shortfall - 80.0) * 0.20;
                    selection_score += std::max(0.0, vt_shortfall - 80.0) * 0.14;
                    if (speed_shortfall > 250.0 || vt_shortfall > 250.0 || orbit.rp_km < orbit_target.rp_km - 40.0) {
                        selection_score += rem_prop * 0.035;
                    }
                }
                const bool better =
                    selection_score < best_selection_score - 1e-9 ||
                    (std::abs(selection_score - best_selection_score) <= 1e-9 &&
                        ((orbit_ok && used < best_used - 1e-6) || (!orbit_ok && used > best_used + 1e-6)));
                if (better) {
                    best_penalty = penalty;
                    best_selection_score = selection_score;
                    best_state = s;
                    best_orbit = orbit;
                    best_cutoff_s = t_global;
                    best_used = used;
                    best_rp_err_km = rp_err_km;
                    best_ra_err_km = ra_err_km;
                    best_target_r_err_km = target_r_err_km;
                    best_fpa_err_deg = fpa_err_deg;
                    best_peak_alt_km = peak_alt_km;
                    best_orbit_ok = orbit_ok;
                    best_converged = cmd.converged || orbit_ok;
                    best_tgo = cmd.tgo_s;
                    best_vgo = cmd.vgo_mps;
                    best_traj_size = cand.traj.size();
                }
                if (orbit_ok && penalty <= 1e-6) {
                    break;
                }
            }
        }

        if (crashed && best_penalty == std::numeric_limits<double>::infinity()) {
            best_state = s;
            best_cutoff_s = t_global;
            best_used = used;
            best_orbit = orbit_metrics_from_state(s);
            best_peak_alt_km = peak_alt_km;
            best_penalty = std::numeric_limits<double>::infinity();
            best_converged = false;
            best_orbit_ok = false;
            best_tgo = upfg_prev_tgo;
            best_vgo = std::hypot(best_state.vr - upfg_target.target_vr_mps, best_state.vt - upfg_target.target_vt_mps);
            best_traj_size = cand.traj.size();
        } else if (!crashed && best_penalty == std::numeric_limits<double>::infinity()) {
            best_state = s;
            best_cutoff_s = t_global;
            best_used = used;
            best_orbit = orbit_metrics_from_state(s);
            best_peak_alt_km = peak_alt_km;
            best_penalty = stage2_orbit_penalty(cand.ignition_s > 0.0 ? orbit_target : orbit_target, Stage2Result{{}, best_state, best_orbit, cand.ignition_s, burn_elapsed, best_cutoff_s, best_used, std::max(0.0, request.s2_prop_kg - best_used)}, best_rp_err_km, best_ra_err_km, best_target_r_err_km, best_fpa_err_deg, best_orbit_ok);
            best_converged = false;
            best_tgo = upfg_prev_tgo;
            best_vgo = std::hypot(best_state.vr - upfg_target.target_vr_mps, best_state.vt - upfg_target.target_vt_mps);
            best_traj_size = cand.traj.size();
        }

        cand.seco = best_state;
        cand.cutoff_s = best_cutoff_s;
        cand.burn_s = cand.cutoff_s - cand.ignition_s;
        cand.used_prop = best_used;
        cand.rem_prop = std::max(0.0, request.s2_prop_kg - best_used);
        cand.orbit = best_orbit;
        cand.target_rp_err_km = best_rp_err_km;
        cand.target_ra_err_km = best_ra_err_km;
        cand.target_r_err_km = best_target_r_err_km;
        cand.target_fpa_err_deg = best_fpa_err_deg;
        cand.peak_alt_km = best_peak_alt_km;
        cand.orbit_penalty = crashed ? std::numeric_limits<double>::infinity() : best_penalty;
        cand.orbit_ok = (!crashed) && best_orbit_ok;
        cand.converged = (!crashed) && best_converged;
        cand.tgo_final_s = best_tgo;
        cand.vgo_final_mps = best_vgo;
        while (cand.traj.size() > best_traj_size) cand.traj.pop_back();
        if (!cand.traj.empty() && std::abs(cand.traj.back().t - cand.cutoff_s) > 1e-6) {
            cand.traj.push_back({cand.cutoff_s, cand.seco.theta * (kRe / 1000.0), std::max(0.0, (cand.seco.r - kRe) / 1000.0)});
        }
        return cand;
    };

    struct Candidate {
        double gamma_bias_deg;
        double climb_start_u;
        double climb_end_u;
        double vr_gain;
        double vt_gain;
        double end_gamma_deg;
        double rate_limit_deg_s;
    };

    const Candidate candidates[] = {
        {0.0, 0.18, 0.92, 1.30, 1.45, -0.2, 0.90},
        {-3.5, 0.26, 0.98, 1.05, 1.75, -0.9, 1.25},
        {-2.5, 0.24, 0.96, 1.10, 1.65, -0.6, 1.20},
        {-1.5, 0.20, 0.94, 1.15, 1.60, -0.5, 1.10},
        {-0.5, 0.18, 0.93, 1.25, 1.55, -0.3, 1.00},
        {1.0, 0.14, 0.90, 1.45, 1.30, 0.0, 0.90},
        {2.0, 0.12, 0.88, 1.60, 1.20, 0.2, 0.85},
        {3.0, 0.10, 0.88, 1.90, 1.00, 0.6, 0.80},
        {4.2, 0.08, 0.86, 2.10, 0.90, 1.0, 0.75},
        {5.0, 0.10, 0.96, 2.30, 0.95, 1.8, 0.80},
        {6.5, 0.14, 1.00, 2.50, 0.90, 2.4, 0.85},
    };

    Stage2Result best = simulate_candidate(
        candidates[0].gamma_bias_deg,
        candidates[0].climb_start_u,
        candidates[0].climb_end_u,
        candidates[0].vr_gain,
        candidates[0].vt_gain,
        candidates[0].end_gamma_deg,
        candidates[0].rate_limit_deg_s);

    for (size_t i = 1; i < std::size(candidates); ++i) {
        const Candidate& c = candidates[i];
        Stage2Result cand = simulate_candidate(c.gamma_bias_deg, c.climb_start_u, c.climb_end_u, c.vr_gain, c.vt_gain, c.end_gamma_deg, c.rate_limit_deg_s);
        const bool better =
            cand.orbit_penalty < best.orbit_penalty - 1e-9 ||
            (std::abs(cand.orbit_penalty - best.orbit_penalty) <= 1e-9 && cand.used_prop < best.used_prop - 1e-6);
        if (better) best = std::move(cand);
    }

    return best;
}

bool cancellation_requested(SolveControl control) {
    return control.cancel_requested && control.cancel_requested->load(std::memory_order_relaxed);
}

double finite_or(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

double candidate_q_excess(const SeparationCandidate& cand, const MissionRequest& request) {
    return std::max(0.0, finite_or(cand.stage1.max_q, request.q_limit_kpa + 1000.0) - request.q_limit_kpa);
}

double candidate_sep_error_score(const SeparationCandidate& cand) {
    return
        finite_or(std::abs(cand.stage1.target_alt_err_km), 1e6) * 4.0 +
        finite_or(std::abs(cand.stage1.target_speed_err_mps), 1e6) * 0.02 +
        finite_or(std::abs(cand.stage1.target_gamma_err_deg), 1e6) * 8.0;
}

double candidate_recovery_deficit(const SeparationCandidate& cand) {
    return
        std::max(0.0, -finite_or(cand.recovery.margin_kg, -1e6)) +
        2000.0 * std::max(0.0, finite_or(cand.recovery.touchdown_speed_mps, 1e6) - 10.0);
}

double candidate_stage2_reserve_kg(const SeparationCandidate& cand) {
    return finite_or(cand.stage2.rem_prop, -1e12);
}

double candidate_stage2_reserve_score(const SeparationCandidate& cand, const MissionRequest& request) {
    return finite_or(cand.stage2.rem_prop, 0.0) / std::max(1.0, request.s2_prop_kg);
}

bool candidate_mission_accepted(const SeparationCandidate& cand, bool recovery_required) {
    return cand.stage1.converged &&
           cand.stage2.converged &&
           cand.stage2.orbit_ok &&
           (!recovery_required || cand.recovery.feasible);
}

bool candidate_recovery_seed_better(const SeparationCandidate& cand, const SeparationCandidate& best, const MissionRequest& request) {
    if (!std::isfinite(cand.score)) return false;
    if (!std::isfinite(best.score)) return true;

    const bool cand_recovery = cand.recovery.feasible;
    const bool best_recovery = best.recovery.feasible;
    if (cand_recovery != best_recovery) return cand_recovery;

    if (cand.recovery.margin_kg > best.recovery.margin_kg + 1e-6) return true;
    if (cand.recovery.margin_kg < best.recovery.margin_kg - 1e-6) return false;

    const double cand_recovery_deficit = candidate_recovery_deficit(cand);
    const double best_recovery_deficit = candidate_recovery_deficit(best);
    if (cand_recovery_deficit < best_recovery_deficit - 1e-6) return true;
    if (cand_recovery_deficit > best_recovery_deficit + 1e-6) return false;

    const double cand_q_excess = candidate_q_excess(cand, request);
    const double best_q_excess = candidate_q_excess(best, request);
    if (cand_q_excess < best_q_excess - 1e-6) return true;
    if (cand_q_excess > best_q_excess + 1e-6) return false;

    return cand.score < best.score;
}

bool candidate_orbit_seed_better(const SeparationCandidate& cand, const SeparationCandidate& best, const MissionRequest& request) {
    if (!std::isfinite(cand.score)) return false;
    if (!std::isfinite(best.score)) return true;

    const bool cand_orbit = cand.stage1.converged && cand.stage2.converged && cand.stage2.orbit_ok;
    const bool best_orbit = best.stage1.converged && best.stage2.converged && best.stage2.orbit_ok;
    if (cand_orbit != best_orbit) return cand_orbit;

    if (cand.orbit_miss_score < best.orbit_miss_score - 1e-9) return true;
    if (cand.orbit_miss_score > best.orbit_miss_score + 1e-9) return false;

    const double cand_q_excess = candidate_q_excess(cand, request);
    const double best_q_excess = candidate_q_excess(best, request);
    if (cand_q_excess < best_q_excess - 1e-6) return true;
    if (cand_q_excess > best_q_excess + 1e-6) return false;

    const double cand_sep_error = candidate_sep_error_score(cand);
    const double best_sep_error = candidate_sep_error_score(best);
    if (cand_sep_error < best_sep_error - 1e-6) return true;
    if (cand_sep_error > best_sep_error + 1e-6) return false;

    const bool cand_recovery = cand.recovery.feasible;
    const bool best_recovery = best.recovery.feasible;
    if (cand_recovery != best_recovery) return cand_recovery;

    const double cand_recovery_deficit = candidate_recovery_deficit(cand);
    const double best_recovery_deficit = candidate_recovery_deficit(best);
    if (cand_recovery_deficit < best_recovery_deficit - 1e-6) return true;
    if (cand_recovery_deficit > best_recovery_deficit + 1e-6) return false;

    return cand.score < best.score;
}

bool candidate_fuel_seed_better(const SeparationCandidate& cand, const SeparationCandidate& best, const MissionRequest& request) {
    if (!std::isfinite(cand.score)) return false;
    if (!std::isfinite(best.score)) return true;

    const bool cand_numeric = cand.stage1.converged && std::isfinite(cand.stage2.orbit_penalty);
    const bool best_numeric = best.stage1.converged && std::isfinite(best.stage2.orbit_penalty);
    if (cand_numeric != best_numeric) return cand_numeric;

    const bool cand_orbit = cand.stage1.converged && cand.stage2.converged && cand.stage2.orbit_ok;
    const bool best_orbit = best.stage1.converged && best.stage2.converged && best.stage2.orbit_ok;
    if (cand_orbit != best_orbit) return cand_orbit;

    const double cand_stage2_reserve = candidate_stage2_reserve_kg(cand);
    const double best_stage2_reserve = candidate_stage2_reserve_kg(best);
    if (cand_stage2_reserve > best_stage2_reserve + 1e-6) return true;
    if (cand_stage2_reserve < best_stage2_reserve - 1e-6) return false;

    const bool cand_recovery = cand.recovery.feasible;
    const bool best_recovery = best.recovery.feasible;
    if (cand_recovery != best_recovery) return cand_recovery;

    if (cand.orbit_miss_score < best.orbit_miss_score - 1e-9) return true;
    if (cand.orbit_miss_score > best.orbit_miss_score + 1e-9) return false;

    const double cand_recovery_deficit = candidate_recovery_deficit(cand);
    const double best_recovery_deficit = candidate_recovery_deficit(best);
    if (cand_recovery_deficit < best_recovery_deficit - 1e-6) return true;
    if (cand_recovery_deficit > best_recovery_deficit + 1e-6) return false;

    return cand.score < best.score;
}

bool candidate_better(
    const SeparationCandidate& cand,
    const SeparationCandidate& best,
    const MissionRequest& request,
    bool require_accepted = true,
    bool recovery_required = true) {
    if (!std::isfinite(cand.score)) return false;
    if (require_accepted && !candidate_mission_accepted(cand, recovery_required)) return false;
    if (!std::isfinite(best.score)) return true;
    if (require_accepted && !candidate_mission_accepted(best, recovery_required)) return true;

    const bool cand_numeric = cand.stage1.converged && std::isfinite(cand.stage2.orbit_penalty);
    const bool best_numeric = best.stage1.converged && std::isfinite(best.stage2.orbit_penalty);
    if (cand_numeric != best_numeric) return cand_numeric;

    const bool cand_recovery = cand.recovery.feasible;
    const bool best_recovery = best.recovery.feasible;
    if (cand_recovery != best_recovery) return cand_recovery;

    if (cand_numeric && cand_recovery) {
        const bool cand_orbit = cand.stage2.orbit_ok;
        const bool best_orbit = best.stage2.orbit_ok;
        if (cand_orbit != best_orbit) return cand_orbit;
        const double cand_stage2_reserve = candidate_stage2_reserve_kg(cand);
        const double best_stage2_reserve = candidate_stage2_reserve_kg(best);
        if (cand_stage2_reserve > best_stage2_reserve + 1e-6) return true;
        if (cand_stage2_reserve < best_stage2_reserve - 1e-6) return false;
        if (cand.orbit_miss_score < best.orbit_miss_score - 1e-9) return true;
        if (cand.orbit_miss_score > best.orbit_miss_score + 1e-9) return false;
        const double cand_q_excess = candidate_q_excess(cand, request);
        const double best_q_excess = candidate_q_excess(best, request);
        if (cand_q_excess < best_q_excess - 1e-6) return true;
        if (cand_q_excess > best_q_excess + 1e-6) return false;
        const double cand_sep_error = candidate_sep_error_score(cand);
        const double best_sep_error = candidate_sep_error_score(best);
        if (cand_sep_error < best_sep_error - 1e-6) return true;
        if (cand_sep_error > best_sep_error + 1e-6) return false;
        return cand.score < best.score;
    }

    const double cand_recovery_deficit = candidate_recovery_deficit(cand);
    const double best_recovery_deficit = candidate_recovery_deficit(best);
    if (cand_recovery_deficit < best_recovery_deficit - 1e-6) return true;
    if (cand_recovery_deficit > best_recovery_deficit + 1e-6) return false;
    if (cand.orbit_miss_score < best.orbit_miss_score - 1e-9) return true;
    if (cand.orbit_miss_score > best.orbit_miss_score + 1e-9) return false;
    const double cand_q_excess = candidate_q_excess(cand, request);
    const double best_q_excess = candidate_q_excess(best, request);
    if (cand_q_excess < best_q_excess - 1e-6) return true;
    if (cand_q_excess > best_q_excess + 1e-6) return false;
    const double cand_sep_error = candidate_sep_error_score(cand);
    const double best_sep_error = candidate_sep_error_score(best);
    if (cand_sep_error < best_sep_error - 1e-6) return true;
    if (cand_sep_error > best_sep_error + 1e-6) return false;
    return cand.score < best.score;
}

struct SepTimeEvaluation {
    SeparationCandidate accepted_best;
    SeparationCandidate orbit_seed_best;
    SeparationCandidate fuel_seed_best;
    SeparationCandidate relaxed_best;
    SeparationCandidate recovery_seed_best;
};

SepTimeEvaluation evaluate_sep_time_candidate(
    const MissionRequest& request,
    const OrbitTarget& orbit_target,
    const std::vector<double>& sep_alt_grid,
    const std::vector<double>& sep_speed_grid,
    const std::vector<double>& sep_gamma_grid,
    double sep_time_s,
    SolveControl control) {
    SepTimeEvaluation best;
    const bool recovery_required = !control.ignore_recovery;
    for (double sep_alt_km : sep_alt_grid) {
        if (cancellation_requested(control)) break;
        for (double sep_speed_mps : sep_speed_grid) {
            if (cancellation_requested(control)) break;
            for (double sep_gamma_deg : sep_gamma_grid) {
                if (cancellation_requested(control)) break;
                const Stage1Target target{
                    sep_time_s,
                    sep_alt_km,
                    sep_speed_mps,
                    sep_gamma_deg,
                    // When recovery is intentionally ignored (e.g. forced burnout
                    // sweep) the ascent burn is allowed to consume the full S1
                    // propellant load.  Otherwise we hold the configured landing
                    // reserve (s1_reserve) untouched so simulate_stage1_recovery
                    // has fuel to work with.
                    !recovery_required || control.force_stage1_burnout,
                };
                SeparationCandidate cand;
                cand.sep_time_s = sep_time_s;
                cand.sep_alt_target_km = sep_alt_km;
                cand.sep_speed_target_mps = sep_speed_mps;
                cand.sep_gamma_target_deg = sep_gamma_deg;
                cand.stage1 = simulate_stage1_candidate(request, target);
                cand.stage2 = simulate_stage2_candidate(request, cand.stage1, orbit_target);
                if (recovery_required) {
                    cand.recovery = simulate_stage1_recovery(request, cand.stage1, orbit_target.launch_az_deg);
                } else {
                    cand.recovery.feasible = true;
                    cand.recovery.converged = true;
                    cand.recovery.margin_kg = 0.0;
                }

                const bool numeric_ok = cand.stage1.converged && cand.stage2.converged;
                const bool orbit_ok = cand.stage2.orbit_ok;
                const bool recovery_ok = cand.recovery.feasible;
                cand.feasible = numeric_ok && orbit_ok && recovery_ok;

                cand.orbit_miss_score = finite_or(cand.stage2.orbit_penalty, 1e12);
                cand.recovery_surplus_kg = std::max(0.0, finite_or(cand.recovery.margin_kg, 0.0));
                const double recovery_deficit = candidate_recovery_deficit(cand);
                cand.score =
                    cand.orbit_miss_score * 1000.0 +
                    recovery_deficit * 180.0 +
                    candidate_q_excess(cand, request) * 200.0 +
                    candidate_sep_error_score(cand) -
                    candidate_stage2_reserve_score(cand, request) * 1000.0;
                if (candidate_better(cand, best.accepted_best, request, true, recovery_required)) best.accepted_best = cand;
                if (candidate_orbit_seed_better(cand, best.orbit_seed_best, request)) best.orbit_seed_best = cand;
                if (candidate_fuel_seed_better(cand, best.fuel_seed_best, request)) best.fuel_seed_best = cand;
                if (candidate_recovery_seed_better(cand, best.recovery_seed_best, request)) best.recovery_seed_best = cand;
                if (candidate_better(cand, best.relaxed_best, request, false, recovery_required)) best.relaxed_best = std::move(cand);
            }
        }
    }
    return best;
}

unsigned resolve_worker_count(SolveControl control, size_t sample_count) {
    unsigned workers = control.worker_count;
    if (workers == 0) {
        workers = std::thread::hardware_concurrency();
        if (workers == 0) workers = 1;
    }
    workers = std::max(1u, std::min<unsigned>(workers, 9u));
    workers = std::min<unsigned>(workers, static_cast<unsigned>(std::max<size_t>(1, sample_count)));
    return workers;
}

struct SepSearchResult {
    SeparationCandidate accepted_best;
    SeparationCandidate orbit_seed_best;
    SeparationCandidate fuel_seed_best;
    SeparationCandidate relaxed_best;
    SeparationCandidate recovery_seed_best;
};

SepSearchResult search_sep_samples(
    const MissionRequest& request,
    const OrbitTarget& orbit_target,
    const std::vector<double>& sep_alt_grid,
    const std::vector<double>& sep_speed_grid,
    const std::vector<double>& sep_gamma_grid,
    const std::vector<double>& sep_times,
    SolveControl control,
    std::vector<SeparationCandidate>* sample_out = nullptr) {
    if (sep_times.empty()) return {};

    std::vector<SepTimeEvaluation> sample_best(sep_times.size());
    const unsigned workers = resolve_worker_count(control, sep_times.size());
    const bool recovery_required = !control.ignore_recovery;

    auto worker_fn = [&](unsigned worker_index) {
        for (size_t i = worker_index; i < sep_times.size(); i += workers) {
            if (cancellation_requested(control)) break;
            sample_best[i] = evaluate_sep_time_candidate(
                request,
                orbit_target,
                sep_alt_grid,
                sep_speed_grid,
                sep_gamma_grid,
                sep_times[i],
                control);
        }
    };

    std::vector<std::thread> threads;
    threads.reserve(workers > 1 ? workers - 1 : 0);
    for (unsigned worker_index = 1; worker_index < workers; ++worker_index) {
        threads.emplace_back(worker_fn, worker_index);
    }
    worker_fn(0);
    for (std::thread& thread : threads) {
        thread.join();
    }

    SepSearchResult best;
    for (const SepTimeEvaluation& cand : sample_best) {
        if (candidate_better(cand.accepted_best, best.accepted_best, request, true, recovery_required)) best.accepted_best = cand.accepted_best;
        if (candidate_orbit_seed_better(cand.orbit_seed_best, best.orbit_seed_best, request)) best.orbit_seed_best = cand.orbit_seed_best;
        if (candidate_fuel_seed_better(cand.fuel_seed_best, best.fuel_seed_best, request)) best.fuel_seed_best = cand.fuel_seed_best;
        if (candidate_recovery_seed_better(cand.recovery_seed_best, best.recovery_seed_best, request)) best.recovery_seed_best = cand.recovery_seed_best;
        if (candidate_better(cand.relaxed_best, best.relaxed_best, request, false, recovery_required)) best.relaxed_best = cand.relaxed_best;
    }
    if (sample_out) {
        for (const SepTimeEvaluation& cand : sample_best) {
            if (std::isfinite(cand.accepted_best.score)) {
                sample_out->push_back(cand.accepted_best);
            } else if (std::isfinite(cand.orbit_seed_best.score)) {
                sample_out->push_back(cand.orbit_seed_best);
            } else if (std::isfinite(cand.fuel_seed_best.score)) {
                sample_out->push_back(cand.fuel_seed_best);
            } else if (std::isfinite(cand.recovery_seed_best.score)) {
                sample_out->push_back(cand.recovery_seed_best);
            } else if (std::isfinite(cand.relaxed_best.score)) {
                sample_out->push_back(cand.relaxed_best);
            }
        }
    }
    return best;
}

void append_separation_time_series(MissionResult& result, std::vector<SeparationCandidate> samples) {
    if (samples.empty()) return;

    std::sort(samples.begin(), samples.end(), [](const SeparationCandidate& a, const SeparationCandidate& b) {
        if (a.sep_time_s != b.sep_time_s) return a.sep_time_s < b.sep_time_s;
        return a.score < b.score;
    });

    Series s2_remaining;
    s2_remaining.name = L"Stage2 Remaining Propellant";
    s2_remaining.color = RGB(142, 68, 173);

    Series s1_reserve;
    s1_reserve.name = L"Stage1 Reserve Propellant";
    s1_reserve.color = RGB(211, 84, 0);

    Series landing_margin;
    landing_margin.name = L"Landing Margin";
    landing_margin.color = RGB(39, 174, 96);

    Series landing_prop;
    landing_prop.name = L"Landing Propellant";
    landing_prop.color = RGB(192, 57, 43);

    double last_sep_time = -std::numeric_limits<double>::infinity();
    for (const SeparationCandidate& cand : samples) {
        if (!std::isfinite(cand.sep_time_s) || cand.sep_time_s == last_sep_time) continue;
        last_sep_time = cand.sep_time_s;
        result.separation_candidates.push_back(cand);
        const SeparationCandidate& kept = result.separation_candidates.back();
        if (std::isfinite(kept.stage2.rem_prop)) {
            s2_remaining.pts.push_back({kept.sep_time_s, kept.stage2.rem_prop});
        }
        if (std::isfinite(kept.stage1.rem_prop)) {
            s1_reserve.pts.push_back({kept.sep_time_s, kept.stage1.rem_prop});
        }
        if (std::isfinite(kept.recovery.margin_kg)) {
            landing_margin.pts.push_back({kept.sep_time_s, kept.recovery.margin_kg});
        }
        if (std::isfinite(kept.recovery.landing_prop_kg)) {
            landing_prop.pts.push_back({kept.sep_time_s, kept.recovery.landing_prop_kg});
        }
    }

    if (!s2_remaining.pts.empty()) result.separation_time_series.push_back(std::move(s2_remaining));
    if (!s1_reserve.pts.empty()) result.separation_time_series.push_back(std::move(s1_reserve));
    if (!landing_margin.pts.empty()) result.separation_time_series.push_back(std::move(landing_margin));
    if (!landing_prop.pts.empty()) result.separation_time_series.push_back(std::move(landing_prop));
}

GlobeSeries profile_to_globe(const Series& src, const MissionRequest& request, double launch_az_deg, double launch_lon_deg) {
    GlobeSeries gs;
    gs.name = src.name;
    gs.color = src.color;
    gs.pts.reserve(src.pts.size());
    for (const PlotPt& q : src.pts) {
        if (!finite_plot_pt(q)) continue;
        GlobePt gp;
        destination_from_course(
            request.lat_deg,
            launch_lon_deg,
            launch_az_deg,
            std::max(0.0, q.x_km),
            gp.lat_deg,
            gp.lon_deg);
        gp.alt_km = std::max(0.0, q.y_km);
        if (finite_globe_pt(gp)) gs.pts.push_back(gp);
    }
    return gs;
}

void append_post_orbit_series(MissionResult& result, const MissionRequest& request) {
    const Stage2Result& stage2 = result.stage2;
    if (!stage2.orbit_ok) return;

    GlobePt insertion_gp{};
    destination_from_course(
        request.lat_deg,
        result.launch_lon_deg,
        result.orbit_target.launch_az_deg,
        std::max(0.0, stage2.seco.theta * (kRe / 1000.0)),
        insertion_gp.lat_deg,
        insertion_gp.lon_deg);
    insertion_gp.alt_km = std::max(0.0, (stage2.seco.r - kRe) / 1000.0);

    const Vec3 rhat = normalize3(ecef_from_geo(insertion_gp.lat_deg, insertion_gp.lon_deg, 0.0));
    const Vec3 east = normalize3({-std::sin(deg2rad(insertion_gp.lon_deg)), std::cos(deg2rad(insertion_gp.lon_deg)), 0.0});
    const Vec3 north = normalize3({
        -std::sin(deg2rad(insertion_gp.lat_deg)) * std::cos(deg2rad(insertion_gp.lon_deg)),
        -std::sin(deg2rad(insertion_gp.lat_deg)) * std::sin(deg2rad(insertion_gp.lon_deg)),
        std::cos(deg2rad(insertion_gp.lat_deg))});
    const double az_rad = deg2rad(result.orbit_target.launch_az_deg);
    const Vec3 horiz = normalize3({
        north.x * std::cos(az_rad) + east.x * std::sin(az_rad),
        north.y * std::cos(az_rad) + east.y * std::sin(az_rad),
        north.z * std::cos(az_rad) + east.z * std::sin(az_rad),
    });
    const double fpa_rad = deg2rad(stage2.orbit.flight_path_deg);
    const Vec3 vdir = normalize3({
        horiz.x * std::cos(fpa_rad) + rhat.x * std::sin(fpa_rad),
        horiz.y * std::cos(fpa_rad) + rhat.y * std::sin(fpa_rad),
        horiz.z * std::cos(fpa_rad) + rhat.z * std::sin(fpa_rad),
    });

    const double rmag_km = stage2.seco.r / 1000.0;
    const double speed_kmps = stage2.orbit.speed_mps / 1000.0;
    const Vec3 r_vec{rmag_km * rhat.x, rmag_km * rhat.y, rmag_km * rhat.z};
    const Vec3 v_vec{speed_kmps * vdir.x, speed_kmps * vdir.y, speed_kmps * vdir.z};
    const Vec3 h_vec = cross3(r_vec, v_vec);
    const double h_norm = std::sqrt(dot3(h_vec, h_vec));
    if (h_norm <= 1e-8) return;

    const Vec3 h_hat = normalize3(h_vec);
    const Vec3 e_vec = {
        (v_vec.y * h_vec.z - v_vec.z * h_vec.y) / kMuKm - r_vec.x / std::max(1e-9, rmag_km),
        (v_vec.z * h_vec.x - v_vec.x * h_vec.z) / kMuKm - r_vec.y / std::max(1e-9, rmag_km),
        (v_vec.x * h_vec.y - v_vec.y * h_vec.x) / kMuKm - r_vec.z / std::max(1e-9, rmag_km),
    };
    const double e_norm = std::sqrt(dot3(e_vec, e_vec));
    Vec3 u_orb = (e_norm > 1e-8) ? normalize3(e_vec) : normalize3(r_vec);
    Vec3 w_orb = normalize3(cross3(h_hat, u_orb));
    if (dot3(w_orb, w_orb) < 1e-10) {
        w_orb = normalize3(cross3(h_hat, normalize3(r_vec)));
    }
    const double nu0 = std::atan2(dot3(normalize3(r_vec), w_orb), dot3(normalize3(r_vec), u_orb));
    const double p_orb = (h_norm * h_norm) / kMuKm;
    const double e_orb = clampd(stage2.orbit.e, 0.0, 0.99);

    GlobeSeries post_orbit;
    post_orbit.name = L"Post-Insertion Orbit";
    post_orbit.color = RGB(243, 156, 18);
    const int n_orbit = 540;
    post_orbit.pts.reserve(static_cast<size_t>(n_orbit + 1));
    for (int i = 0; i <= n_orbit; ++i) {
        const double nu = nu0 + (2.0 * 3.14159265358979323846 * static_cast<double>(i)) / static_cast<double>(n_orbit);
        const double rmag = p_orb / std::max(1e-8, 1.0 + e_orb * std::cos(nu));
        const Vec3 q{
            rmag * (std::cos(nu) * u_orb.x + std::sin(nu) * w_orb.x),
            rmag * (std::cos(nu) * u_orb.y + std::sin(nu) * w_orb.y),
            rmag * (std::cos(nu) * u_orb.z + std::sin(nu) * w_orb.z),
        };
        const double qn = std::sqrt(dot3(q, q));
        GlobePt gp{};
        gp.lat_deg = rad2deg(std::asin(clampd(q.z / std::max(1e-9, qn), -1.0, 1.0)));
        gp.lon_deg = wrap_lon_deg(rad2deg(std::atan2(q.y, q.x)));
        gp.alt_km = std::max(0.0, qn - (kRe / 1000.0));
        post_orbit.pts.push_back(gp);
    }
    result.globe_series.push_back(std::move(post_orbit));
}

}  // namespace

OrbitMetrics orbit_metrics_from_state(const PolarState& s) {
    OrbitMetrics out;
    out.speed_mps = std::hypot(s.vr, s.vt);
    out.flight_path_deg = rad2deg(std::atan2(s.vr, std::max(1e-9, s.vt)));

    const double r_km = s.r / 1000.0;
    const double vr_km = s.vr / 1000.0;
    const double vt_km = s.vt / 1000.0;
    const double v2 = vr_km * vr_km + vt_km * vt_km;
    const double energy = 0.5 * v2 - kMuKm / std::max(1e-9, r_km);
    const double h = r_km * vt_km;
    const double ecc_sq = std::max(0.0, 1.0 + (2.0 * energy * h * h) / (kMuKm * kMuKm));
    out.e = std::sqrt(ecc_sq);

    if (std::abs(energy) > 1e-12) {
        out.a_km = -kMuKm / (2.0 * energy);
    } else {
        out.a_km = std::numeric_limits<double>::infinity();
    }

    if (std::isfinite(out.a_km) && out.a_km > 0.0) {
        out.rp_km = std::max(0.0, out.a_km * (1.0 - out.e) - (kRe / 1000.0));
        out.ra_km = std::max(0.0, out.a_km * (1.0 + out.e) - (kRe / 1000.0));
    } else {
        out.rp_km = std::max(0.0, r_km - (kRe / 1000.0));
        out.ra_km = out.rp_km;
    }
    return out;
}

OrbitTarget build_orbit_target(const MissionRequest& request) {
    OrbitTarget out;
    out.rp_km = std::min(request.perigee_km, request.apogee_km);
    out.ra_km = std::max(request.perigee_km, request.apogee_km);
    out.cutoff_alt_km = clampd(
        std::isfinite(request.cutoff_alt_km) ? request.cutoff_alt_km : out.rp_km,
        out.rp_km,
        out.ra_km);

    const double lat = deg2rad(request.lat_deg);
    const double inc = deg2rad(std::abs(request.incl_deg));
    const double c_lat = std::cos(lat);
    double sin_az = 0.0;
    if (std::abs(c_lat) > 1e-6) {
        sin_az = clampd(std::cos(inc) / c_lat, -1.0, 1.0);
    }
    out.launch_az_deg = rad2deg(std::asin(sin_az));

    const double rp = kRe + out.rp_km * 1000.0;
    const double ra = kRe + out.ra_km * 1000.0;
    const double r_cut = kRe + out.cutoff_alt_km * 1000.0;
    const double a = 0.5 * (rp + ra);
    const double e = clampd((ra - rp) / std::max(1.0, ra + rp), 0.0, 0.999);
    const double p = a * (1.0 - e * e);
    double nu = 0.0;
    if (e > 1e-8) {
        const double cos_nu = clampd((p / r_cut - 1.0) / e, -1.0, 1.0);
        nu = std::acos(cos_nu);
    }
    const double speed = std::sqrt(kMu * (2.0 / r_cut - 1.0 / a));
    const double fpa = (e > 1e-8) ? std::atan2(e * std::sin(nu), 1.0 + e * std::cos(nu)) : 0.0;
    out.r_target_m = r_cut;
    out.speed_target_mps = speed;
    out.fpa_target_deg = rad2deg(fpa);
    out.vr_target_mps = speed * std::sin(fpa);
    out.vt_target_mps = speed * std::cos(fpa);
    return out;
}

MissionRequest sanitize_request(const MissionRequest& request) {
    MissionRequest out = request;
    if (!std::isfinite(out.cutoff_alt_km)) out.cutoff_alt_km = std::min(out.perigee_km, out.apogee_km);

    out.payload_kg = std::max(0.0, out.payload_kg);
    out.perigee_km = std::max(80.0, out.perigee_km);
    out.apogee_km = std::max(80.0, out.apogee_km);
    if (out.perigee_km > out.apogee_km) std::swap(out.perigee_km, out.apogee_km);
    out.cutoff_alt_km = clampd(out.cutoff_alt_km, out.perigee_km, out.apogee_km);
    out.q_limit_kpa = clampd(out.q_limit_kpa, 20.0, 80.0);
    out.s1_target_maxq_kpa = clampd(out.s1_target_maxq_kpa, 15.0, out.q_limit_kpa);
    out.s1_sep_delay_s = clampd(out.s1_sep_delay_s, 1.5, 6.0);
    out.s2_ignition_delay_s = clampd(out.s2_ignition_delay_s, 3.0, 15.0);
    return out;
}

MissionResult solve_mission(const MissionRequest& request_in, SolveControl control) {
    const MissionRequest request = sanitize_request(request_in);
    const bool recovery_required = !control.ignore_recovery;
    MissionResult result;
    result.orbit_target = build_orbit_target(request);
    result.launch_lat_deg = request.lat_deg;
    result.launch_lon_deg = request.launch_lon_deg;

    const double thrust = request.s1_thrust_kN * 1000.0;
    const double mdot = thrust / std::max(1e-6, request.s1_isp_s * kG0);
    const double burn_max = request.s1_prop_kg / std::max(1e-6, mdot);
    const double sep_delay_s = clampd(request.s1_sep_delay_s, 1.5, 6.0);
    const double sep_lo = std::max(60.0, 0.40 * burn_max + sep_delay_s);
    // Extend the upper search bound so the planner can still find an orbit-feasible
    // candidate when stage 1 must burn close to depletion (e.g. heavy payloads where
    // the only viable insertion happens with a near-burnout separation, like the
    // dedicated burnout/no-recovery mode).  The extra +60 s headroom keeps the burnout
    // sep time inside the search window without shrinking the existing low-payload
    // refinement region.
    const double burnout_sep_time_s = burn_max + sep_delay_s + 90.0;
    const double sep_hi = std::max(
        std::min(220.0, burn_max + sep_delay_s + 65.0),
        std::min(burnout_sep_time_s + 5.0, burn_max + sep_delay_s + 120.0));

    const double alt_center = clampd(52.0 + 0.03 * (result.orbit_target.cutoff_alt_km - 200.0), 42.0, 76.0);
    const double speed_center = clampd(
        3000.0 + 0.90 * (result.orbit_target.cutoff_alt_km - 200.0) + 0.35 * (result.orbit_target.ra_km - result.orbit_target.rp_km),
        2500.0,
        3600.0);
    const double gamma_center = clampd(2.0 + 0.003 * (result.orbit_target.ra_km - result.orbit_target.rp_km), -0.5, 6.0);
    const std::vector<double> sep_alt_grid = {
        clampd(alt_center - 32.0, 10.0, 90.0),
        clampd(alt_center - 24.0, 24.0, 90.0),
        clampd(alt_center - 16.0, 24.0, 90.0),
        clampd(alt_center - 8.0, 24.0, 90.0),
        clampd(alt_center, 24.0, 90.0),
        clampd(alt_center + 6.0, 40.0, 90.0),
        clampd(alt_center + 14.0, 40.0, 90.0),
    };
    const std::vector<double> sep_speed_grid = {
        clampd(speed_center - 1200.0, 1000.0, 3600.0),
        clampd(speed_center - 900.0, 1600.0, 3600.0),
        clampd(speed_center - 600.0, 1900.0, 3600.0),
        clampd(speed_center - 300.0, 2000.0, 3600.0),
        clampd(speed_center, 2000.0, 3600.0),
        clampd(speed_center + 300.0, 2000.0, 3600.0),
        clampd(speed_center + 550.0, 2000.0, 3600.0),
    };
    const std::vector<double> sep_gamma_grid = {
        clampd(gamma_center - 2.5, -1.0, 16.0),
        clampd(gamma_center, -1.0, 16.0),
        clampd(gamma_center + 2.5, -1.0, 16.0),
    };

    std::vector<SeparationCandidate> sep_sweep_samples;
    SeparationCandidate best;
    bool best_is_accepted = false;
    auto promote_best_from_search = [&](const SepSearchResult& search, bool allow_marginal_fallback) {
        if (std::isfinite(search.accepted_best.score)) {
            if (!best_is_accepted || candidate_better(search.accepted_best, best, request, true, recovery_required)) {
                best = search.accepted_best;
                best_is_accepted = true;
            }
            return;
        }
        if (best_is_accepted || !allow_marginal_fallback) return;
        bool showing_orbit_seed = best.stage1.converged && best.stage2.converged && best.stage2.orbit_ok;
        if (candidate_orbit_seed_better(search.orbit_seed_best, best, request)) {
            best = search.orbit_seed_best;
            showing_orbit_seed = best.stage1.converged && best.stage2.converged && best.stage2.orbit_ok;
        }
        bool showing_fuel_seed = false;
        if (!showing_orbit_seed && candidate_fuel_seed_better(search.fuel_seed_best, best, request)) {
            best = search.fuel_seed_best;
            showing_fuel_seed = true;
        }
        if (!showing_orbit_seed && !showing_fuel_seed && candidate_better(search.recovery_seed_best, best, request, false, recovery_required)) best = search.recovery_seed_best;
        if (!showing_orbit_seed && !showing_fuel_seed && candidate_better(search.relaxed_best, best, request, false, recovery_required)) best = search.relaxed_best;
    };

    if (control.force_stage1_burnout) {
        const std::vector<double> burnout_sep_times{burnout_sep_time_s};
        SepSearchResult burnout_search = search_sep_samples(
            request,
            result.orbit_target,
            sep_alt_grid,
            sep_speed_grid,
            sep_gamma_grid,
            burnout_sep_times,
            control,
            &sep_sweep_samples);
        promote_best_from_search(burnout_search, true);
    } else if (control.separation_search_mode == SeparationSearchMode::Coarse1s) {
        constexpr double kCoarseSepStepS = 1.0;
        std::vector<double> coarse_sep_times;
        const double span_s = std::max(0.0, sep_hi - sep_lo);
        coarse_sep_times.reserve(static_cast<size_t>(std::floor(span_s / kCoarseSepStepS)) + 2);
        for (double t = sep_lo; t <= sep_hi + 1e-9; t += kCoarseSepStepS) {
            coarse_sep_times.push_back(std::min(t, sep_hi));
        }
        if (coarse_sep_times.empty() || std::abs(coarse_sep_times.back() - sep_hi) > 1e-6) {
            coarse_sep_times.push_back(sep_hi);
        }

        SepSearchResult coarse_search = search_sep_samples(
            request,
            result.orbit_target,
            sep_alt_grid,
            sep_speed_grid,
            sep_gamma_grid,
            coarse_sep_times,
            control,
            &sep_sweep_samples);
        promote_best_from_search(coarse_search, true);
    } else {
        std::vector<double> coarse_sep_times;
        coarse_sep_times.reserve(11);
        for (int i = 0; i <= 8; ++i) {
            const double u = static_cast<double>(i) / 8.0;
            coarse_sep_times.push_back(lerpd(sep_lo, sep_hi, u));
        }
        // Add the near-burnout separation time explicitly so heavy-payload missions,
        // which can only reach orbit when stage 1 runs close to depletion, are always
        // probed by the coarse sweep (the uniform grid above can miss this narrow
        // window on Falcon 9-like vehicles).
        const double burnout_probe = clampd(burnout_sep_time_s, sep_lo, sep_hi);
        if (std::isfinite(burnout_probe)) {
            bool already_present = false;
            for (double t : coarse_sep_times) {
                if (std::abs(t - burnout_probe) <= 1e-6) {
                    already_present = true;
                    break;
                }
            }
            if (!already_present) coarse_sep_times.push_back(burnout_probe);
        }

        SepSearchResult coarse_search = search_sep_samples(
            request,
            result.orbit_target,
            sep_alt_grid,
            sep_speed_grid,
            sep_gamma_grid,
            coarse_sep_times,
            control,
            &sep_sweep_samples);
        promote_best_from_search(coarse_search, true);
        std::vector<double> refine_seed_times;
        auto add_refine_seed_time = [&](const SeparationCandidate& seed) {
            if (!std::isfinite(seed.score) || !std::isfinite(seed.sep_time_s)) return;
            for (double existing : refine_seed_times) {
                if (std::abs(existing - seed.sep_time_s) <= 1e-6) return;
            }
            refine_seed_times.push_back(seed.sep_time_s);
        };
        add_refine_seed_time(coarse_search.accepted_best);
        add_refine_seed_time(coarse_search.orbit_seed_best);
        add_refine_seed_time(coarse_search.fuel_seed_best);
        add_refine_seed_time(coarse_search.recovery_seed_best);
        add_refine_seed_time(coarse_search.relaxed_best);

        if (refine_seed_times.empty() && !cancellation_requested(control)) {
            const std::vector<double> fallback_sep_time{0.5 * (sep_lo + sep_hi)};
            SepSearchResult fallback_search = search_sep_samples(
                request,
                result.orbit_target,
                sep_alt_grid,
                sep_speed_grid,
                sep_gamma_grid,
                fallback_sep_time,
                control,
                &sep_sweep_samples);
            promote_best_from_search(fallback_search, true);
            add_refine_seed_time(fallback_search.accepted_best);
            add_refine_seed_time(fallback_search.orbit_seed_best);
            add_refine_seed_time(fallback_search.fuel_seed_best);
            add_refine_seed_time(fallback_search.recovery_seed_best);
            add_refine_seed_time(fallback_search.relaxed_best);
        }

        for (double refine_seed_time : refine_seed_times) {
            if (cancellation_requested(control)) break;
            const double refine_lo = std::max(sep_lo, refine_seed_time - 4.0);
            const double refine_hi = std::min(sep_hi, refine_seed_time + 4.0);
            std::vector<double> refine_sep_times;
            refine_sep_times.reserve(9);
            for (int i = 0; i <= 8; ++i) {
                const double u = static_cast<double>(i) / 8.0;
                refine_sep_times.push_back(lerpd(refine_lo, refine_hi, u));
            }

            SepSearchResult refined = search_sep_samples(
                request,
                result.orbit_target,
                sep_alt_grid,
                sep_speed_grid,
                sep_gamma_grid,
                refine_sep_times,
                control,
                &sep_sweep_samples);
            promote_best_from_search(refined, false);
        }
    }

    if (!std::isfinite(best.score)) {
        result.status = L"MARGINAL / NOT FEASIBLE";
        result.payload_search_ok = false;
        result.ok = false;
        result.lines.push_back(L"[Status] " + result.status);
        result.lines.push_back(
            L"[Orbit Target] rp=" + fnum(result.orbit_target.rp_km, 1) +
            L" km, ra=" + fnum(result.orbit_target.ra_km, 1) +
            L" km, cutoff=" + fnum(result.orbit_target.cutoff_alt_km, 1) +
            L" km, i=" + fnum(request.incl_deg, 2) + L" deg");
        result.lines.push_back(L"[Reserve Fuel] Stage1=N/A kg, Stage2=N/A kg");
        result.lines.push_back(L"[Reserve Objective] maximize Stage2 remaining propellant = N/A");
        if (recovery_required) {
            result.lines.push_back(L"[Search] No candidate satisfied both Stage2 orbit insertion and Stage1 landing ignition constraints.");
            result.lines.push_back(L"[Search] Discarded candidates with orbit_ok=0 or recovery_feasible=0.");
        } else {
            result.lines.push_back(L"[Search] No candidate satisfied Stage2 orbit insertion constraints.");
            result.lines.push_back(L"[Search] Recovery constraints were ignored for this mode.");
        }
        return result;
    }

    result.best_candidate = best;
    result.stage1 = best.stage1;
    result.stage2 = best.stage2;
    result.recovery = best.recovery;
    append_separation_time_series(result, std::move(sep_sweep_samples));
    if (std::isfinite(best.score) && recovery_required) {
        result.ship_lat_deg = best.recovery.touchdown_lat_deg;
        result.ship_lon_deg = best.recovery.touchdown_lon_deg;
        result.view_lat_deg = 0.5 * (result.launch_lat_deg + result.ship_lat_deg);
        result.view_lon_deg = wrap_lon_deg(result.launch_lon_deg + 0.5 * (result.ship_lon_deg - result.launch_lon_deg));
    } else {
        result.ship_lat_deg = result.launch_lat_deg;
        result.ship_lon_deg = result.launch_lon_deg;
        result.view_lat_deg = result.launch_lat_deg;
        result.view_lon_deg = result.launch_lon_deg;
    }

    const bool numeric_ok = result.stage1.converged && result.stage2.converged;
    const bool orbit_ok = result.stage2.orbit_ok;
    const bool recovery_ok = (!recovery_required) || result.recovery.feasible;
    const bool q_ok = result.stage1.max_q <= request.q_limit_kpa + 1e-6;
    result.payload_search_ok = numeric_ok && orbit_ok && recovery_ok;
    result.ok = result.payload_search_ok && q_ok;

    if (result.ok) {
        result.status = recovery_required ? L"FEASIBLE" : L"FEASIBLE / NO RECOVERY";
    } else if (result.payload_search_ok) {
        result.status = recovery_required ? L"FEASIBLE / Q-LIMIT" : L"FEASIBLE / NO RECOVERY / Q-LIMIT";
    } else if (recovery_required && numeric_ok && orbit_ok && !result.recovery.feasible) {
        // Stage 2 reached orbit (UPFG insertion succeeded), but Stage 1 cannot land
        // with the propellant left after burn-out separation.  Surface this as a
        // distinct status so the GUI still shows the insertion trajectory instead of
        // a generic "not feasible" message.
        result.status = q_ok ? L"ORBIT OK / RECOVERY INFEASIBLE"
                              : L"ORBIT OK / RECOVERY INFEASIBLE / Q-LIMIT";
    } else {
        result.status = L"MARGINAL / NOT FEASIBLE";
    }

    const double sep_alt = (result.stage1.sep.r - kRe) / 1000.0;
    const double sep_speed = std::hypot(result.stage1.sep.vr, result.stage1.sep.vt);
    const double sep_gamma = rad2deg(std::atan2(result.stage1.sep.vr, std::max(1.0, result.stage1.sep.vt)));

    result.lines.push_back(L"[Status] " + result.status);
    result.lines.push_back(
        L"[Orbit Target] rp=" + fnum(result.orbit_target.rp_km, 1) +
        L" km, ra=" + fnum(result.orbit_target.ra_km, 1) +
        L" km, cutoff=" + fnum(result.orbit_target.cutoff_alt_km, 1) +
        L" km, i=" + fnum(request.incl_deg, 2) + L" deg");
    if (!recovery_required) {
        result.lines.push_back(L"[Mode] No recovery; Stage1 burns to depletion before separation.");
    }
    result.lines.push_back(
        L"[Stage1 UPFG Search] sep=" + fnum(best.sep_time_s, 1) +
        L" s, tgt_alt=" + fnum(best.sep_alt_target_km, 1) +
        L" km, tgt_speed=" + fnum(best.sep_speed_target_mps, 0) +
        L" m/s, tgt_gamma=" + fnum(best.sep_gamma_target_deg, 1) + L" deg");
    if (!best_is_accepted) {
        if (best.stage2.orbit_ok) {
            result.lines.push_back(L"[Search] No accepted candidate; showing best Stage2 orbit candidate for diagnostics.");
        } else {
            result.lines.push_back(L"[Search] No accepted candidate; showing best marginal candidate for diagnostics.");
        }
    }
    result.lines.push_back(
        L"[Stage1 UPFG] MECO=" + fnum(result.stage1.meco_s, 1) +
        L" s, separation=" + fnum(result.stage1.sep_s, 1) +
        L" s, sep_alt=" + fnum(sep_alt, 2) +
        L" km, sep_speed=" + fnum(sep_speed, 1) +
        L" m/s, sep_gamma=" + fnum(sep_gamma, 2) + L" deg");
    result.lines.push_back(
        L"[Stage1 Guidance] guide_start=" + fnum(result.stage1.guide_start_s, 1) +
        L" s, tgo_final=" + fnum(result.stage1.tgo_final_s, 1) +
        L" s, vgo_final=" + fnum(result.stage1.vgo_final_mps, 1) +
        L" m/s, maxQ=" + fnum(result.stage1.max_q, 2) + L" kPa");
    result.lines.push_back(
        L"[Stage1 Errors] alt=" + fnum(result.stage1.target_alt_err_km, 2) +
        L" km, speed=" + fnum(result.stage1.target_speed_err_mps, 1) +
        L" m/s, gamma=" + fnum(result.stage1.target_gamma_err_deg, 2) + L" deg");
    result.lines.push_back(
        L"[Stage2 UPFG] ignition=" + fnum(result.stage2.ignition_s, 1) +
        L" s, burn=" + fnum(result.stage2.burn_s, 1) +
        L" s, SECO=" + fnum(result.stage2.cutoff_s, 1) +
        L" s, tgo_final=" + fnum(result.stage2.tgo_final_s, 1) +
        L" s, vgo_final=" + fnum(result.stage2.vgo_final_mps, 1) + L" m/s");
    result.lines.push_back(
        L"[Stage2 Orbit] actual_rp=" + fnum(result.stage2.orbit.rp_km, 1) +
        L" km, actual_ra=" + fnum(result.stage2.orbit.ra_km, 1) +
        L" km, actual_cutoff_alt=" + fnum((result.stage2.seco.r - kRe) / 1000.0, 1) +
        L" km, flight_path=" + fnum(result.stage2.orbit.flight_path_deg, 2) + L" deg");
    result.lines.push_back(
        L"[Stage2 Errors] rp=" + fnum(result.stage2.target_rp_err_km, 1) +
        L" km, ra=" + fnum(result.stage2.target_ra_err_km, 1) +
        L" km, cutoff=" + fnum(result.stage2.target_r_err_km, 1) +
        L" km, fpa=" + fnum(result.stage2.target_fpa_err_deg, 2) + L" deg");
    result.lines.push_back(
        L"[Reserve Fuel] Stage1=" + fnum(result.stage1.rem_prop, 1) +
        L" kg, Stage2=" + fnum(result.stage2.rem_prop, 1) + L" kg");
    result.lines.push_back(
        L"[Reserve Objective] maximize Stage2 remaining propellant = " +
        fnum(result.stage2.rem_prop, 1) + L" kg (" +
        fnum(100.0 * candidate_stage2_reserve_score(best, request), 3) + L"%)");
    if (recovery_required) {
        result.lines.push_back(
            L"[Recovery Coast] ignition=" +
            (std::isfinite(result.recovery.landing_ignition_time_s) ? ftime(result.recovery.landing_ignition_time_s) : std::wstring(L"N/A")) +
            L", touchdown=" + ftime(result.recovery.touchdown_time_s) +
            L", touchdown_downrange=" + fnum(result.recovery.touchdown_downrange_km, 1) + L" km");
        result.lines.push_back(
            L"[Landing Burn] prop=" + fnum(result.recovery.landing_prop_kg, 1) +
            L" kg, margin=" + fnum(result.recovery.margin_kg, 1) +
            L" kg, touchdown_speed=" + fnum(result.recovery.touchdown_speed_mps, 1) + L" m/s");
        result.lines.push_back(
            L"[Recovery Ship LL] lat=" + fnum(result.recovery.touchdown_lat_deg, 6) +
            L", lon=" + fnum(result.recovery.touchdown_lon_deg, 6) +
            L" (propagated touchdown)");
        result.lines.push_back(
            L"[Compatibility] input ship_downrange_km=" + fnum(request.ship_downrange_km, 1) +
            L" km retained for parsing only; automatic ship placement now follows propagated touchdown.");
    } else {
        result.lines.push_back(L"[Recovery] Ignored; Stage1 is expended after burnout separation.");
    }
    result.lines.push_back(L"[Timeline] " + ftime(0.0) + L" Liftoff");
    result.lines.push_back(L"[Timeline] " + ftime(result.stage1.t_max_q) + L" Max-Q");
    result.lines.push_back(L"[Timeline] " + ftime(result.stage1.meco_s) + L" MECO");
    result.lines.push_back(L"[Timeline] " + ftime(result.stage1.sep_s) + L" Stage Separation");
    result.lines.push_back(L"[Timeline] " + ftime(result.stage2.ignition_s) + L" Stage2 Ignition");
    result.lines.push_back(L"[Timeline] " + ftime(result.stage2.cutoff_s) + L" Stage2 SECO");
    if (recovery_required && std::isfinite(result.recovery.landing_ignition_time_s)) {
        result.lines.push_back(L"[Timeline] " + ftime(result.recovery.landing_ignition_time_s) + L" Landing Ignition");
    }
    if (recovery_required) {
        result.lines.push_back(L"[Timeline] " + ftime(result.recovery.touchdown_time_s) + L" Touchdown");
    }

    Series ascent;
    ascent.name = L"Stage1 Ascent";
    ascent.color = RGB(41, 128, 185);
    for (const SimPt& pt : result.stage1.traj) ascent.pts.push_back({pt.x_km, pt.z_km});
    result.profile_series.push_back(std::move(ascent));

    if (recovery_required) {
        Series coast;
        coast.name = L"Recovery Coast";
        coast.color = RGB(39, 174, 96);
        for (const SimPt& pt : result.recovery.coast_traj) coast.pts.push_back({pt.x_km, pt.z_km});
        result.profile_series.push_back(std::move(coast));

        if (!result.recovery.landing_traj.empty()) {
            Series landing;
            landing.name = L"Landing Burn";
            landing.color = RGB(22, 160, 133);
            for (const SimPt& pt : result.recovery.landing_traj) landing.pts.push_back({pt.x_km, pt.z_km});
            result.profile_series.push_back(std::move(landing));
        }
    }

    Series insertion;
    insertion.name = L"Stage2 Insertion";
    insertion.color = RGB(192, 57, 43);
    for (const SimPt& pt : result.stage2.traj) insertion.pts.push_back({pt.x_km, pt.z_km});
    result.profile_series.push_back(std::move(insertion));

    for (const Series& series : result.profile_series) {
        result.globe_series.push_back(profile_to_globe(series, request, result.orbit_target.launch_az_deg, result.launch_lon_deg));
    }
    append_post_orbit_series(result, request);

    return result;
}

}  // namespace falcon9
