#include "planner_mission.hpp"

#include "planner_lvd.hpp"
#include "planner_recovery.hpp"
#include "planner_upfg.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>

namespace falcon9 {

namespace {

using ProfileClock = std::chrono::steady_clock;

struct ProfileCounter {
    std::atomic<long long> ns{0};
    std::atomic<int> count{0};

    void reset() {
        ns.store(0, std::memory_order_relaxed);
        count.store(0, std::memory_order_relaxed);
    }

    void add(ProfileClock::duration dt) {
        ns.fetch_add(std::chrono::duration_cast<std::chrono::nanoseconds>(dt).count(), std::memory_order_relaxed);
        count.fetch_add(1, std::memory_order_relaxed);
    }
};

struct MissionProfile {
    bool enabled = false;
    ProfileCounter solve_total;
    ProfileCounter lvd_total;
    ProfileCounter lvd_mission_search;
    ProfileCounter final_sep_search;
    ProfileCounter sep_candidate_total;
    ProfileCounter clip_stage1;
    ProfileCounter stage2;
    ProfileCounter recovery;
    ProfileCounter report_build;

    void reset(bool on) {
        enabled = on;
        solve_total.reset();
        lvd_total.reset();
        lvd_mission_search.reset();
        final_sep_search.reset();
        sep_candidate_total.reset();
        clip_stage1.reset();
        stage2.reset();
        recovery.reset();
        report_build.reset();
    }
};

MissionProfile& mission_profile() {
    static MissionProfile profile;
    return profile;
}

bool mission_profile_requested() {
    char value[8]{};
    const DWORD n = GetEnvironmentVariableA("F9_PROFILE", value, static_cast<DWORD>(sizeof(value)));
    return n > 0 && value[0] != '0';
}

struct ScopedProfile {
    ProfileCounter* counter = nullptr;
    ProfileClock::time_point start{};

    explicit ScopedProfile(ProfileCounter& c, bool enabled) {
        if (enabled) {
            counter = &c;
            start = ProfileClock::now();
        }
    }

    ~ScopedProfile() {
        if (counter) counter->add(ProfileClock::now() - start);
    }
};

double profile_ms(const ProfileCounter& counter) {
    return static_cast<double>(counter.ns.load(std::memory_order_relaxed)) / 1.0e6;
}

int profile_count(const ProfileCounter& counter) {
    return counter.count.load(std::memory_order_relaxed);
}

void print_mission_profile(const MissionProfile& profile) {
    if (!profile.enabled) return;
    const double total_ms = std::max(1e-9, profile_ms(profile.solve_total));
    auto line = [&](const char* name, const ProfileCounter& c) {
        const double ms = profile_ms(c);
        std::fprintf(
            stderr,
            "[F9_PROFILE] %-24s wall_or_sum=%9.2f ms  pct_of_solve=%6.2f  count=%d\n",
            name,
            ms,
            100.0 * ms / total_ms,
            profile_count(c));
    };
    std::fprintf(stderr, "[F9_PROFILE] ---- solve_mission timing ----\n");
    line("solve_total", profile.solve_total);
    line("stage1_lvd_total", profile.lvd_total);
    line("lvd_nested_sep_search", profile.lvd_mission_search);
    line("final_sep_search", profile.final_sep_search);
    line("sep_candidate_sum", profile.sep_candidate_total);
    line("clip_stage1_sum", profile.clip_stage1);
    line("stage2_upfg_sum", profile.stage2);
    line("recovery_sum", profile.recovery);
    line("report_build", profile.report_build);
    std::fprintf(stderr, "[F9_PROFILE] -------------------------------\n");
}

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

void append_orbit_target_report(
    std::vector<std::wstring>& lines,
    const MissionRequest& request,
    const OrbitTarget& target) {
    const double min_direct_incl = direct_launch_min_incl_deg(request.lat_deg);
    const double max_direct_incl = 180.0 - min_direct_incl;
    const double direct_incl = direct_launch_effective_incl_deg(request.lat_deg, target.launch_az_deg);
    const double requested_incl = clampd(std::abs(request.incl_deg), 0.0, 180.0);

    lines.push_back(
        L"[Orbit Target] rp=" + fnum(target.rp_km, 1) +
        L" km, ra=" + fnum(target.ra_km, 1) +
        L" km, cutoff=" + fnum(target.cutoff_alt_km, 1) +
        L" km, requested_i=" + fnum(request.incl_deg, 2) +
        L" deg, direct_i=" + fnum(direct_incl, 2) +
        L" deg, launch_az=" + fnum(target.launch_az_deg, 2) + L" deg");

    if (requested_incl < min_direct_incl - 1e-6 || requested_incl > max_direct_incl + 1e-6) {
        lines.push_back(
            L"[Orbit Target] requested inclination is outside the direct-launch range " +
            fnum(min_direct_incl, 2) + L".." + fnum(max_direct_incl, 2) +
            L" deg for launch latitude " + fnum(request.lat_deg, 2) +
            L" deg; using the nearest direct plane.");
    }
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
    penalty += std::abs(rp_err_km) * 1.2;
    penalty += std::abs(ra_err_km) * 0.9;
    penalty += std::abs(target_r_err_km) * 1.5;
    penalty += std::abs(fpa_err_deg) * 24.0;
    penalty += std::max(0.0, std::abs(rp_err_km) - 10.0) * 18.0;
    penalty += std::max(0.0, std::abs(ra_err_km) - 16.0) * 10.0;
    penalty += std::max(0.0, std::abs(fpa_err_deg) - 0.35) * 36.0;
    penalty += std::max(0.0, std::abs(target_r_err_km) - 4.0) * 8.0;
    if (stage2.orbit.rp_km < 80.0) penalty += (80.0 - stage2.orbit.rp_km) * 45.0;

    orbit_ok =
        std::abs(rp_err_km) <= 40.0 &&
        std::abs(ra_err_km) <= 60.0 &&
        std::abs(target_r_err_km) <= 40.0 &&
        stage2.orbit.rp_km >= 80.0;
    return penalty;
}

bool stage2_precision_good(
    double rp_err_km,
    double ra_err_km,
    double target_r_err_km,
    double fpa_err_deg) {
    return
        std::abs(rp_err_km) <= 0.75 &&
        std::abs(ra_err_km) <= 1.25 &&
        std::abs(target_r_err_km) <= 0.50 &&
        std::abs(fpa_err_deg) <= 0.05;
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
        const double c_lat = std::cos(lat);
        const double sin_az = direct_launch_sin_az(request.lat_deg, request.incl_deg);
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
            // Max-payload experiment: do not hand stage 1 to UPFG late in the burn.
            // Keep the early pitch/gravity-turn program, then use a small ZEM/ZEV-like
            // terminal correction to nudge the booster toward the requested separation
            // altitude, speed, and flight-path angle.  In the current planner this
            // retains much more stage-1 recovery propellant than the UPFG handoff.
            const double target_z = target.sep_alt_km * 1000.0;
            const double target_vz = target.sep_speed_mps * std::sin(deg2rad(target.sep_gamma_deg));
            const double target_vx = target.sep_speed_mps * std::cos(deg2rad(target.sep_gamma_deg));
            const double tgo = clampd(burn_s - t, 6.0, 80.0);
            const double ax_need = clampd((target_vx - s.vx) / tgo, -25.0, 25.0);
            const double az_need = clampd(
                (target_vz - s.vz) / tgo +
                    2.0 * (target_z - s.z - s.vz * tgo) / std::max(1.0, tgo * tgo),
                -25.0,
                35.0);
            const double thrust_acc_max = thrust / std::max(1.0, s.m);
            const double thrust_ax_cmd = ax_need + drag_ax;
            const double thrust_az_cmd = az_need + g + drag_az;
            gamma_raw = std::atan2(std::max(0.0, thrust_az_cmd), std::max(1e-4, thrust_ax_cmd));
            const double acc_cmd = std::hypot(thrust_ax_cmd, thrust_az_cmd);
            throttle_cmd = clampd(acc_cmd / std::max(1e-6, thrust_acc_max), 0.35, 1.0);
            out.tgo_final_s = tgo;
            out.vgo_final_mps = std::hypot(target_vx - s.vx, target_vz - s.vz);
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

double downrange_from_state3d(const StateVector3D& s, const OrbitTarget& orbit_target) {
    const Vec3 rhat = normalize3(s.r_m);
    return kRe * std::atan2(
        dot3(rhat, orbit_target.launch_tangent_eci),
        dot3(rhat, orbit_target.launch_rhat_eci));
}

Vec3 plane_radial_at(const StateVector3D& s, const OrbitTarget& orbit_target) {
    const Vec3 n_hat = normalize3(orbit_target.plane_normal_eci);
    Vec3 rhat = normalize3(reject3(s.r_m, n_hat));
    if (dot3(rhat, rhat) < 1e-10) rhat = normalize3(s.r_m);
    return rhat;
}

Vec3 plane_tangent_at(const StateVector3D& s, const OrbitTarget& orbit_target) {
    const Vec3 rhat = plane_radial_at(s, orbit_target);
    Vec3 that = normalize3(cross3(normalize3(orbit_target.plane_normal_eci), rhat));
    if (dot3(that, s.v_mps) < 0.0) that *= -1.0;
    return that;
}

Vec3 clamp_thrust_dir_gamma(const Vec3& dir, const Vec3& rhat, double gamma_lo_rad, double gamma_hi_rad) {
    const double radial = dot3(dir, rhat);
    Vec3 horizontal = dir - rhat * radial;
    const double horizontal_norm = norm3(horizontal);
    if (horizontal_norm <= 1e-9) {
        horizontal = reject3({1.0, 0.0, 0.0}, rhat);
        if (norm3(horizontal) <= 1e-9) horizontal = reject3({0.0, 1.0, 0.0}, rhat);
        horizontal = normalize3(horizontal);
    } else {
        horizontal = horizontal / horizontal_norm;
    }

    const double gamma = std::atan2(radial, std::max(1e-9, horizontal_norm));
    const double clamped = clampd(gamma, gamma_lo_rad, gamma_hi_rad);
    return normalize3(horizontal * std::cos(clamped) + rhat * std::sin(clamped));
}

Vec3 limit_direction_change(const Vec3& from, const Vec3& to, double max_angle_rad) {
    const Vec3 a = normalize3(from);
    const Vec3 b = normalize3(to);
    const double c = clampd(dot3(a, b), -1.0, 1.0);
    const double angle = std::acos(c);
    if (angle <= std::max(1e-9, max_angle_rad)) return b;

    Vec3 lateral = b - a * c;
    const double lateral_norm = norm3(lateral);
    if (lateral_norm <= 1e-9) return b;
    lateral = lateral / lateral_norm;
    return normalize3(a * std::cos(max_angle_rad) + lateral * std::sin(max_angle_rad));
}

PolarState polar_from_state3d(const StateVector3D& s, const OrbitTarget& orbit_target) {
    const Vec3 rhat = normalize3(s.r_m);
    const Vec3 that = plane_tangent_at(s, orbit_target);
    return {
        norm3(s.r_m),
        downrange_from_state3d(s, orbit_target) / kRe,
        dot3(s.v_mps, rhat),
        dot3(s.v_mps, that),
        s.m_kg,
    };
}

Stage2Result simulate_stage2_candidate_3d(
    const MissionRequest& request,
    const Stage1Result& stage1,
    const OrbitTarget& orbit_target) {
    Stage2Result out;
    if (!stage1.sep3d.valid || !orbit_target.has_3d_plane) return out;

    StateVector3D ignition_seed = stage1.sep3d;
    ignition_seed.m_kg = request.s2_dry_kg + request.s2_prop_kg + request.payload_kg;
    ignition_seed.valid = true;

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
    const UpfgTarget3D upfg_target{
        orbit_target.r_target_m,
        orbit_target.vr_target_mps,
        orbit_target.vt_target_mps,
        orbit_target.plane_normal_eci,
    };
    const UpfgSettings upfg_settings{
        6.0,
        -10.0,
        55.0,
        1.18,
        1.28,
        1.12,
        10.0,
        14.0,
    };

    auto push_traj = [&](Stage2Result& cand, double t_global, const StateVector3D& s) {
        const double downrange_km = downrange_from_state3d(s, orbit_target) / 1000.0;
        const double alt_km = std::max(0.0, (norm3(s.r_m) - kRe) / 1000.0);
        cand.traj.push_back({t_global, downrange_km, alt_km});
        cand.traj3d.push_back({t_global, s.r_m, s.v_mps});
    };

    auto simulate_candidate = [&](double gamma_bias_deg, double climb_start_u, double climb_end_u, double vr_gain, double vt_gain, double end_gamma_deg, double rate_limit_deg_s) {
        Stage2Result cand;
        cand.traj.reserve(960);
        cand.traj3d.reserve(960);

        StateVector3D s = ignition_seed;
        push_traj(cand, stage1.sep_s, s);
        double t_global = stage1.sep_s;
        double t_local = 0.0;
        while (t_local < ignition_delay - 1e-9) {
            const double dt = std::min(0.5, ignition_delay - t_local);
            propagate_state3d_coast(s, dt);
            t_local += dt;
            t_global += dt;
            if (t_global - cand.traj.back().t >= 1.0 || t_local >= ignition_delay - 1e-6) {
                push_traj(cand, t_global, s);
            }
        }

        cand.ignition3d = s;
        cand.ignition = polar_from_state3d(s, orbit_target);
        cand.ignition_s = t_global;

        const Vec3 rhat_initial = normalize3(s.r_m);
        const Vec3 that_initial = plane_tangent_at(s, orbit_target);
        const double gamma_initial = clampd(
            std::atan2(dot3(s.v_mps, rhat_initial), std::max(1.0, dot3(s.v_mps, that_initial))),
            deg2rad(-2.0),
            deg2rad(45.0));
        Vec3 thrust_dir_cmd = normalize3(that_initial * std::cos(gamma_initial) + rhat_initial * std::sin(gamma_initial));

        double used = 0.0;
        double burn_elapsed = 0.0;
        StateVector3D best_state = s;
        OrbitMetrics best_orbit{};
        double best_cutoff_s = cand.ignition_s;
        double best_used = 0.0;
        double best_rp_err_km = 0.0;
        double best_ra_err_km = 0.0;
        double best_target_r_err_km = 0.0;
        double best_fpa_err_deg = 0.0;
        double best_peak_alt_km = std::max(0.0, (norm3(s.r_m) - kRe) / 1000.0);
        double best_penalty = std::numeric_limits<double>::infinity();
        double best_selection_score = std::numeric_limits<double>::infinity();
        bool best_orbit_ok = false;
        bool best_converged = false;
        double best_tgo = burn_max;
        double best_vgo = std::numeric_limits<double>::infinity();
        size_t best_traj_size = cand.traj.size();
        double peak_alt_km = std::max(0.0, (norm3(s.r_m) - kRe) / 1000.0);
        bool crashed = false;

        while (burn_elapsed < burn_max - 1e-9 && used < request.s2_prop_kg - 1e-9) {
            const double r_now = norm3(s.r_m);
            const double target_r_distance = std::abs(target_r - r_now);
            const double prop_tgo_full = std::max(0.0, (request.s2_prop_kg - used) / std::max(1e-6, mdot_nom));
            double guide_dt = 0.50;
            if (burn_elapsed >= 0.70 * burn_max || target_r_distance <= 120000.0) guide_dt = 0.25;
            if (burn_elapsed >= 0.90 * burn_max || target_r_distance <= 60000.0 || prop_tgo_full <= 35.0) guide_dt = 0.10;
            const double dt = std::min(guide_dt, std::max(0.0, prop_tgo_full));
            if (dt <= 1e-9) break;

            const Vec3 rhat = normalize3(s.r_m);
            const UpfgCommand cmd = upfg_compute_command_3d(s, upfg_vehicle, upfg_target, upfg_settings, prop_tgo_full, dt);
            const double vt_target = target_h / std::max(1.0, r_now);
            const double r_safe = std::max(1.0, r_now);
            const Vec3 gravity = s.r_m * (-kMu / (r_safe * r_safe * r_safe));
            const double alt_to_go = std::max(0.0, target_r - r_now);
            Vec3 thrust_dir_desired = normalize3(cmd.thrust_dir_eci);
            const double rate_limit_deg =
                (alt_to_go > 60000.0) ? 6.0 :
                (alt_to_go > 25000.0) ? 4.5 :
                5.0;
            thrust_dir_cmd = limit_direction_change(thrust_dir_cmd, thrust_dir_desired, deg2rad(rate_limit_deg) * dt);
            Vec3 thrust_dir = thrust_dir_cmd;

            const double throttle_cmd = clampd(cmd.throttle, 0.0, 1.0);
            const double dm = std::min(mdot_nom * throttle_cmd * dt, request.s2_prop_kg - used);
            const double mdot = dm / std::max(1e-9, dt);
            const double thrust_now = mdot * request.s2_isp_s * kG0;
            const Vec3 accel = thrust_dir * (thrust_now / std::max(1.0, s.m_kg)) + gravity;
            s.v_mps += accel * dt;
            s.r_m += s.v_mps * dt;
            s.m_kg -= dm;
            used += dm;
            burn_elapsed += dt;
            t_global += dt;
            if (norm3(s.r_m) < kRe) {
                crashed = true;
                break;
            }
            if (t_global - cand.traj.back().t >= 1.0 || burn_elapsed >= burn_max - 1e-6) {
                push_traj(cand, t_global, s);
            }
            peak_alt_km = std::max(peak_alt_km, std::max(0.0, (norm3(s.r_m) - kRe) / 1000.0));

            if (burn_elapsed >= 90.0 && (std::abs(norm3(s.r_m) - target_r) <= 90000.0 || burn_elapsed >= 0.65 * burn_max || cmd.tgo_s <= 85.0)) {
                const OrbitMetrics orbit = orbit_metrics_from_state3d(s);
                double rp_err_km = 0.0;
                double ra_err_km = 0.0;
                double target_r_err_km = 0.0;
                double fpa_err_deg = 0.0;
                bool orbit_ok = false;
                const PolarState probe_polar = polar_from_state3d(s, orbit_target);
                const Stage2Result probe_state{
                    {},
                    probe_polar,
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
                    const double vt_shortfall = std::max(0.0, vt_target - dot3(s.v_mps, plane_tangent_at(s, orbit_target)));
                    selection_score += std::max(0.0, speed_shortfall - 80.0) * 0.20;
                    selection_score += std::max(0.0, vt_shortfall - 80.0) * 0.14;
                    if (speed_shortfall > 250.0 || vt_shortfall > 250.0 || orbit.rp_km < orbit_target.rp_km - 40.0) {
                        const double rp_deficit_km = std::max(0.0, orbit_target.rp_km - orbit.rp_km);
                        selection_score += rem_prop * clampd(0.035 + 0.0006 * rp_deficit_km, 0.035, 0.38);
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
                if (stage2_precision_good(rp_err_km, ra_err_km, target_r_err_km, fpa_err_deg)) break;
            }
        }

        if (best_penalty == std::numeric_limits<double>::infinity()) {
            best_state = s;
            best_cutoff_s = t_global;
            best_used = used;
            best_orbit = orbit_metrics_from_state3d(s);
            best_peak_alt_km = peak_alt_km;
            best_penalty = crashed ? std::numeric_limits<double>::infinity() : stage2_orbit_penalty(
                orbit_target,
                Stage2Result{{}, polar_from_state3d(s, orbit_target), {}, s, best_orbit, cand.ignition_s, burn_elapsed, best_cutoff_s, best_used, std::max(0.0, request.s2_prop_kg - best_used)},
                best_rp_err_km,
                best_ra_err_km,
                best_target_r_err_km,
                best_fpa_err_deg,
                best_orbit_ok);
            best_converged = false;
            best_tgo = std::max(0.0, (request.s2_prop_kg - best_used) / std::max(1e-6, mdot_nom));
            Vec3 target_v = normalize3(best_state.r_m) * upfg_target.target_vr_mps + plane_tangent_at(best_state, orbit_target) * upfg_target.target_vt_mps;
            best_vgo = norm3(best_state.v_mps - target_v);
            best_traj_size = cand.traj.size();
        }

        cand.seco3d = best_state;
        cand.seco3d.valid = true;
        cand.seco = polar_from_state3d(best_state, orbit_target);
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
        while (cand.traj3d.size() > best_traj_size) cand.traj3d.pop_back();
        if (!cand.traj.empty() && std::abs(cand.traj.back().t - cand.cutoff_s) > 1e-6) {
            push_traj(cand, cand.cutoff_s, cand.seco3d);
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
        {-6.0, 0.34, 1.00, 0.85, 2.35, -3.2, 1.80},
        {-4.8, 0.30, 0.99, 0.95, 2.10, -2.3, 1.55},
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

    Stage2Result best;
    best.orbit_penalty = std::numeric_limits<double>::infinity();
    bool have_best = false;
    for (const Candidate& c : candidates) {
        Stage2Result cand = simulate_candidate(c.gamma_bias_deg, c.climb_start_u, c.climb_end_u, c.vr_gain, c.vt_gain, c.end_gamma_deg, c.rate_limit_deg_s);
        if (!have_best ||
            cand.orbit_penalty < best.orbit_penalty - 1e-9 ||
            (std::abs(cand.orbit_penalty - best.orbit_penalty) <= 1e-9 && cand.rem_prop > best.rem_prop + 1e-6)) {
            best = std::move(cand);
            have_best = true;
        }
        if (best.orbit_ok && stage2_precision_good(best.target_rp_err_km, best.target_ra_err_km, best.target_r_err_km, best.target_fpa_err_deg)) break;
    }
    return best;
}

Stage2Result simulate_stage2_candidate(
    const MissionRequest& request,
    const Stage1Result& stage1,
    const OrbitTarget& orbit_target) {
    if (stage1.sep3d.valid && orbit_target.has_3d_plane) {
        Stage2Result out3d = simulate_stage2_candidate_3d(request, stage1, orbit_target);
        if (out3d.converged || std::isfinite(out3d.orbit_penalty)) return out3d;
    }
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
        -10.0,
        55.0,
        1.18,
        1.28,
        1.12,
        10.0,
        14.0,
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
            const double target_r_distance = std::abs(target_r - s.r);
            double guide_dt = 0.50;
            if (burn_elapsed >= 0.70 * burn_max || target_r_distance <= 120000.0) guide_dt = 0.25;
            if (burn_elapsed >= 0.90 * burn_max || target_r_distance <= 60000.0 || upfg_prev_tgo <= 35.0) guide_dt = 0.10;
            const double dt = std::min(guide_dt, burn_max - burn_elapsed);
            const double t_go = std::max(4.0, burn_max - burn_elapsed);
            const double gamma_now = std::atan2(s.vr, std::max(1.0, s.vt));
            const UpfgCommand cmd = upfg_compute_command(s, upfg_vehicle, upfg_target, upfg_settings, upfg_prev_tgo, dt);
            upfg_prev_tgo = cmd.tgo_s;
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
            double upfg_blend = 0.0;
            if (std::isfinite(cmd.gamma_cmd_rad) && std::isfinite(cmd.tgo_s)) {
                const double upfg_time_blend = smoothstep(0.30 * burn_max, 0.84 * burn_max, burn_elapsed);
                const double upfg_radius_blend = smoothstep(target_r - 90000.0, target_r - 4000.0, s.r);
                const double upfg_tgo_blend = 1.0 - smoothstep(28.0, 85.0, cmd.tgo_s);
                upfg_blend = clampd(std::max(upfg_time_blend, std::max(upfg_radius_blend, upfg_tgo_blend)), 0.0, 0.92);
                gamma_raw = lerpd(gamma_raw, cmd.gamma_cmd_rad, upfg_blend);
            }
            const double gamma_hi_deg =
                (alt_to_go > 60000.0) ? 75.0 :
                (alt_to_go > 25000.0) ? 60.0 :
                (alt_to_go > 8000.0) ? 35.0 : 22.0;
            const double gamma_lo_deg = (alt_to_go < 25000.0 || cmd.tgo_s < 45.0) ? -10.0 : -5.0;
            gamma_raw = clampd(gamma_raw, deg2rad(gamma_lo_deg), deg2rad(gamma_hi_deg));
            const double rate_limit_deg =
                (alt_to_go > 60000.0) ? std::max(rate_limit_deg_s, 5.0) :
                (alt_to_go > 25000.0) ? std::max(rate_limit_deg_s, 3.5) :
                std::max(rate_limit_deg_s, lerpd(1.8, 4.0, upfg_blend));
            const double rate_limit = deg2rad(rate_limit_deg) * dt;
            gamma_cmd += clampd(gamma_raw - gamma_cmd, -rate_limit, rate_limit);

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

            if (burn_elapsed >= 90.0 && (std::abs(s.r - target_r) <= 90000.0 || burn_elapsed >= 0.65 * burn_max || cmd.tgo_s <= 85.0)) {
                const OrbitMetrics orbit = orbit_metrics_from_state(s);
                double rp_err_km = 0.0;
                double ra_err_km = 0.0;
                double target_r_err_km = 0.0;
                double fpa_err_deg = 0.0;
                bool orbit_ok = false;
                const Stage2Result probe_state{
                    {},
                    s,
                    {},
                    {},
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
                        const double rp_deficit_km = std::max(0.0, orbit_target.rp_km - orbit.rp_km);
                        const double burn_on_weight = clampd(0.035 + 0.0006 * rp_deficit_km, 0.035, 0.38);
                        selection_score += rem_prop * burn_on_weight;
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
                if (stage2_precision_good(rp_err_km, ra_err_km, target_r_err_km, fpa_err_deg)) {
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
            best_penalty = stage2_orbit_penalty(cand.ignition_s > 0.0 ? orbit_target : orbit_target, Stage2Result{{}, best_state, {}, {}, best_orbit, cand.ignition_s, burn_elapsed, best_cutoff_s, best_used, std::max(0.0, request.s2_prop_kg - best_used)}, best_rp_err_km, best_ra_err_km, best_target_r_err_km, best_fpa_err_deg, best_orbit_ok);
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
        {-6.0, 0.34, 1.00, 0.85, 2.35, -3.2, 1.80},
        {-4.8, 0.30, 0.99, 0.95, 2.10, -2.3, 1.55},
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

LvdStateSample interpolate_lvd_state_sample(const std::vector<LvdStateSample>& samples, double t_s) {
    if (samples.empty()) return {};
    if (t_s <= samples.front().t_s) return samples.front();
    if (t_s >= samples.back().t_s) return samples.back();

    for (size_t i = 1; i < samples.size(); ++i) {
        if (samples[i].t_s < t_s) continue;
        const LvdStateSample& a = samples[i - 1];
        const LvdStateSample& b = samples[i];
        const double u = clampd((t_s - a.t_s) / std::max(1e-9, b.t_s - a.t_s), 0.0, 1.0);
        LvdStateSample out;
        out.t_s = t_s;
        out.state.r = lerpd(a.state.r, b.state.r, u);
        out.state.theta = lerpd(a.state.theta, b.state.theta, u);
        out.state.vr = lerpd(a.state.vr, b.state.vr, u);
        out.state.vt = lerpd(a.state.vt, b.state.vt, u);
        out.state.m = lerpd(a.state.m, b.state.m, u);
        out.state3d.valid = a.state3d.valid && b.state3d.valid;
        if (out.state3d.valid) {
            out.state3d.r_m = a.state3d.r_m + (b.state3d.r_m - a.state3d.r_m) * u;
            out.state3d.v_mps = a.state3d.v_mps + (b.state3d.v_mps - a.state3d.v_mps) * u;
            out.state3d.m_kg = lerpd(a.state3d.m_kg, b.state3d.m_kg, u);
        }
        out.q_kpa = lerpd(a.q_kpa, b.q_kpa, u);
        out.throttle = lerpd(a.throttle, b.throttle, u);
        return out;
    }
    return samples.back();
}

Stage1Result clip_stage1_from_lvd(
    const MissionRequest& request,
    const OrbitTarget& orbit_target,
    const LvdResult& lvd,
    double sep_time_s) {
    Stage1Result out;
    if (lvd.state_samples.empty()) return out;

    const double sep_delay_s = clampd(request.s1_sep_delay_s, 1.5, 6.0);
    const double meco_time_s = clampd(sep_time_s - sep_delay_s, 0.0, lvd.stage1.meco_s);
    const LvdStateSample meco_sample = interpolate_lvd_state_sample(lvd.state_samples, meco_time_s);

    out.guide_start_s = lvd.stage1.guide_start_s;
    out.min_throttle = 1.0;
    out.t_min_throttle = 0.0;
    out.traj.reserve(lvd.state_samples.size() + 16);
    out.traj3d.reserve(lvd.state_samples.size() + 16);

    for (const LvdStateSample& s : lvd.state_samples) {
        if (s.t_s > meco_time_s + 1e-9) break;
        const double x_km = s.state.theta * (kRe / 1000.0);
        const double z_km = std::max(0.0, (s.state.r - kRe) / 1000.0);
        if (out.traj.empty() || s.t_s - out.traj.back().t >= 1.0 || std::abs(s.t_s - meco_time_s) <= 1e-6) {
            out.traj.push_back({s.t_s, x_km, z_km});
            if (s.state3d.valid) out.traj3d.push_back({s.t_s, s.state3d.r_m, s.state3d.v_mps});
        }
        if (s.q_kpa > out.max_q) {
            out.max_q = s.q_kpa;
            out.t_max_q = s.t_s;
        }
        if (s.throttle < out.min_throttle - 1e-9) {
            out.min_throttle = s.throttle;
            out.t_min_throttle = s.t_s;
        }
    }

    const double meco_x_km = meco_sample.state.theta * (kRe / 1000.0);
    const double meco_z_km = std::max(0.0, (meco_sample.state.r - kRe) / 1000.0);
    if (out.traj.empty() || std::abs(out.traj.back().t - meco_time_s) > 1e-6) {
        out.traj.push_back({meco_time_s, meco_x_km, meco_z_km});
    }
    if (meco_sample.q_kpa > out.max_q) {
        out.max_q = meco_sample.q_kpa;
        out.t_max_q = meco_time_s;
    }
    if (meco_sample.throttle < out.min_throttle - 1e-9) {
        out.min_throttle = meco_sample.throttle;
        out.t_min_throttle = meco_time_s;
    }

    const double initial_mass =
        request.s1_dry_kg + request.s1_prop_kg +
        request.s2_dry_kg + request.s2_prop_kg + request.payload_kg;
    out.meco = meco_sample.state;
    out.meco3d = meco_sample.state3d;
    out.meco_s = meco_time_s;
    out.burn_s = meco_time_s;
    out.used_prop = clampd(initial_mass - meco_sample.state.m, 0.0, request.s1_prop_kg);
    out.rem_prop = std::max(0.0, request.s1_prop_kg - out.used_prop);

    if (meco_sample.state3d.valid) {
        StateVector3D s = meco_sample.state3d;
        s.m_kg = meco_sample.state.m;
        const double cda = 9.0;
        double t = meco_time_s;
        double coast_elapsed = 0.0;
        while (coast_elapsed < sep_delay_s - 1e-9) {
            const double dt = std::min(0.20, sep_delay_s - coast_elapsed);
            const double r_norm = std::max(1.0, norm3(s.r_m));
            const double alt_m = std::max(0.0, r_norm - kRe);
            const Vec3 air_v = s.v_mps - cross3({0.0, 0.0, kOmega}, s.r_m);
            const double air_speed = norm3(air_v);
            const double dens = rho(alt_m);
            const double drag = 0.5 * dens * air_speed * air_speed * cda;
            Vec3 drag_acc{0.0, 0.0, 0.0};
            if (air_speed > 1e-6) {
                drag_acc = air_v * ((drag / std::max(1.0, s.m_kg)) / air_speed);
            }
            const Vec3 gravity = s.r_m * (-kMu / (r_norm * r_norm * r_norm));
            s.v_mps += (gravity - drag_acc) * dt;
            s.r_m += s.v_mps * dt;
            t += dt;
            coast_elapsed += dt;
            if (norm3(s.r_m) < kRe) {
                const Vec3 rhat = normalize3(s.r_m);
                s.r_m = rhat * kRe;
                const double vr = dot3(s.v_mps, rhat);
                if (vr < 0.0) s.v_mps -= rhat * vr;
            }

            const Vec3 rhat = normalize3(s.r_m);
            const double downrange_km = kRe * std::atan2(
                dot3(rhat, orbit_target.launch_tangent_eci),
                dot3(rhat, orbit_target.launch_rhat_eci)) / 1000.0;
            const double z_km = std::max(0.0, (norm3(s.r_m) - kRe) / 1000.0);
            if (out.traj.empty() || t - out.traj.back().t >= 1.0 || coast_elapsed >= sep_delay_s - 1e-6) {
                const double legacy_x_km = meco_sample.state.theta * (kRe / 1000.0) +
                    (t - meco_time_s) * meco_sample.state.vt / 1000.0;
                out.traj.push_back({t, std::isfinite(downrange_km) ? downrange_km : legacy_x_km, z_km});
                out.traj3d.push_back({t, s.r_m, s.v_mps});
            }
        }
        out.sep3d = s;
        out.sep3d.valid = true;
        const Vec3 rhat = normalize3(s.r_m);
        const Vec3 n_hat = normalize3(orbit_target.plane_normal_eci);
        Vec3 that = normalize3(cross3(n_hat, rhat));
        if (dot3(that, s.v_mps) < 0.0) that *= -1.0;
        out.sep = {norm3(s.r_m), out.traj.empty() ? meco_sample.state.theta : out.traj.back().x_km * 1000.0 / kRe, dot3(s.v_mps, rhat), dot3(s.v_mps, that), s.m_kg};
        out.sep_s = t;
        const double sep_alt_km = (out.sep.r - kRe) / 1000.0;
        const double sep_speed_mps = norm3(s.v_mps);
        const double sep_gamma_deg = rad2deg(std::atan2(out.sep.vr, std::max(1.0, std::sqrt(std::max(0.0, sep_speed_mps * sep_speed_mps - out.sep.vr * out.sep.vr)))));
        out.target_alt_err_km = sep_alt_km - lvd.target_sep_alt_km;
        out.target_speed_err_mps = sep_speed_mps - lvd.target_sep_speed_mps;
        out.target_gamma_err_deg = sep_gamma_deg - lvd.target_sep_gamma_deg;
        out.tgo_final_s = std::max(0.0, lvd.stage1.sep_s - out.sep_s);
        out.vgo_final_mps = std::hypot(lvd.stage1.sep.vr - out.sep.vr, lvd.stage1.sep.vt - out.sep.vt);
        out.converged =
            std::isfinite(sep_alt_km) &&
            std::isfinite(sep_speed_mps) &&
            std::isfinite(sep_gamma_deg) &&
            std::isfinite(out.max_q);
        out.envelope_ok =
            out.max_q <= request.q_limit_kpa + 1e-6 &&
            sep_alt_km >= 35.0 &&
            sep_alt_km <= 115.0 &&
            sep_speed_mps >= 1500.0 &&
            sep_speed_mps <= 3800.0 &&
            sep_gamma_deg >= -6.0 &&
            sep_gamma_deg <= 22.0;
        return out;
    }

    struct FlatState {
        double x = 0.0;
        double z = 0.0;
        double vx = 0.0;
        double vz = 0.0;
        double m = 0.0;
    };
    FlatState s;
    s.x = meco_sample.state.theta * kRe;
    s.z = std::max(0.0, meco_sample.state.r - kRe);
    s.vx = meco_sample.state.vt;
    s.vz = meco_sample.state.vr;
    s.m = meco_sample.state.m;
    double atmosphere_vx = 0.0;
    {
        const double lat = deg2rad(request.lat_deg);
        const double c_lat = std::cos(lat);
        const double sin_az = direct_launch_sin_az(request.lat_deg, request.incl_deg);
        atmosphere_vx = kOmega * kRe * c_lat * std::max(0.0, sin_az);
    }

    const double cda = 9.0;
    double t = meco_time_s;
    double coast_elapsed = 0.0;
    while (coast_elapsed < sep_delay_s - 1e-9) {
        const double dt = std::min(0.20, sep_delay_s - coast_elapsed);
        const double alt_m = std::max(0.0, s.z);
        const double speed = std::hypot(s.vx, s.vz);
        const double air_vx = s.vx - atmosphere_vx;
        const double air_vz = s.vz;
        const double air_speed = std::hypot(air_vx, air_vz);
        const double dens = rho(alt_m);
        const double g = grav(alt_m);
        const double drag = 0.5 * dens * air_speed * air_speed * cda;
        double drag_ax = 0.0;
        double drag_az = 0.0;
        if (air_speed > 1e-6) {
            const double invm = 1.0 / std::max(1.0, s.m);
            drag_ax = (drag * invm) * (air_vx / air_speed);
            drag_az = (drag * invm) * (air_vz / air_speed);
        }
        double ax = 0.0;
        double az = -g;
        if (air_speed > 1e-6) {
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
        if (out.traj.empty() || t - out.traj.back().t >= 1.0 || coast_elapsed >= sep_delay_s - 1e-6) {
            out.traj.push_back({t, s.x / 1000.0, std::max(0.0, s.z / 1000.0)});
        }
    }

    out.sep = {kRe + std::max(0.0, s.z), s.x / kRe, s.vz, s.vx, s.m};
    out.sep_s = t;
    const double sep_alt_km = (out.sep.r - kRe) / 1000.0;
    const double sep_speed_mps = std::hypot(out.sep.vr, out.sep.vt);
    const double sep_gamma_deg = rad2deg(std::atan2(out.sep.vr, std::max(1.0, out.sep.vt)));
    out.target_alt_err_km = 0.0;
    out.target_speed_err_mps = 0.0;
    out.target_gamma_err_deg = 0.0;
    out.tgo_final_s = std::max(0.0, lvd.stage1.sep_s - out.sep_s);
    out.vgo_final_mps = std::hypot(lvd.stage1.sep.vr - out.sep.vr, lvd.stage1.sep.vt - out.sep.vt);
    out.converged =
        std::isfinite(sep_alt_km) &&
        std::isfinite(sep_speed_mps) &&
        std::isfinite(sep_gamma_deg) &&
        std::isfinite(out.max_q);
    out.envelope_ok =
        out.max_q <= request.q_limit_kpa + 1e-6 &&
        sep_alt_km >= 20.0 &&
        sep_alt_km <= 120.0 &&
        sep_speed_mps >= 900.0 &&
        sep_speed_mps <= 3900.0 &&
        sep_gamma_deg >= -10.0 &&
        sep_gamma_deg <= 30.0;
    return out;
}

SepTimeEvaluation evaluate_lvd_sep_time_candidate(
    const MissionRequest& request,
    const OrbitTarget& orbit_target,
    const LvdResult& lvd,
    double sep_time_s,
    SolveControl control) {
    MissionProfile& profile = mission_profile();
    ScopedProfile profile_scope(profile.sep_candidate_total, profile.enabled);
    SepTimeEvaluation best;
    const bool recovery_required = !control.ignore_recovery;
    if (cancellation_requested(control)) return best;

    SeparationCandidate cand;
    cand.sep_time_s = sep_time_s;
    {
        ScopedProfile clip_scope(profile.clip_stage1, profile.enabled);
        cand.stage1 = clip_stage1_from_lvd(request, orbit_target, lvd, sep_time_s);
    }
    const double sep_alt_km = (cand.stage1.sep.r - kRe) / 1000.0;
    const double sep_speed_mps = std::hypot(cand.stage1.sep.vr, cand.stage1.sep.vt);
    const double sep_gamma_deg = rad2deg(std::atan2(cand.stage1.sep.vr, std::max(1.0, cand.stage1.sep.vt)));
    cand.sep_alt_target_km = sep_alt_km;
    cand.sep_speed_target_mps = sep_speed_mps;
    cand.sep_gamma_target_deg = sep_gamma_deg;

    {
        ScopedProfile stage2_scope(profile.stage2, profile.enabled);
        cand.stage2 = simulate_stage2_candidate(request, cand.stage1, orbit_target);
    }
    if (recovery_required) {
        ScopedProfile recovery_scope(profile.recovery, profile.enabled);
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
    cand.score =
        cand.orbit_miss_score * 1000.0 +
        candidate_recovery_deficit(cand) * 180.0 +
        candidate_q_excess(cand, request) * 200.0 +
        candidate_sep_error_score(cand) -
        candidate_stage2_reserve_score(cand, request) * 1000.0;

    if (candidate_better(cand, best.accepted_best, request, true, recovery_required)) best.accepted_best = cand;
    if (candidate_orbit_seed_better(cand, best.orbit_seed_best, request)) best.orbit_seed_best = cand;
    if (candidate_fuel_seed_better(cand, best.fuel_seed_best, request)) best.fuel_seed_best = cand;
    if (candidate_recovery_seed_better(cand, best.recovery_seed_best, request)) best.recovery_seed_best = cand;
    if (candidate_better(cand, best.relaxed_best, request, false, recovery_required)) best.relaxed_best = std::move(cand);
    return best;
}

SepSearchResult search_lvd_sep_samples(
    const MissionRequest& request,
    const OrbitTarget& orbit_target,
    const LvdResult& lvd,
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
            sample_best[i] = evaluate_lvd_sep_time_candidate(
                request,
                orbit_target,
                lvd,
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

std::vector<double> build_lvd_score_sep_times(const MissionRequest& request, const LvdResult& lvd) {
    std::vector<double> sep_times;
    if (lvd.state_samples.empty() || !std::isfinite(lvd.stage1.sep_s)) return sep_times;

    const double sep_delay_s = clampd(request.s1_sep_delay_s, 1.5, 6.0);
    const double sep_hi = std::max(sep_delay_s, lvd.stage1.sep_s);
    const double sep_lo = std::min(
        sep_hi,
        std::max(sep_delay_s + 60.0, sep_delay_s + 0.55 * std::max(1.0, lvd.stage1.meco_s)));
    sep_times.reserve(5);
    for (int i = 0; i <= 4; ++i) {
        const double u = static_cast<double>(i) / 4.0;
        sep_times.push_back(lerpd(sep_lo, sep_hi, u));
    }
    return sep_times;
}

double score_lvd_search_result(const SepSearchResult& search, bool recovery_required) {
    if (std::isfinite(search.accepted_best.score) &&
        candidate_mission_accepted(search.accepted_best, recovery_required)) {
        return search.accepted_best.score;
    }

    constexpr double relaxed_penalty = 1.0e8;
    auto relaxed_score = [](const SeparationCandidate& cand, double class_penalty) {
        if (!std::isfinite(cand.score)) return std::numeric_limits<double>::infinity();
        return class_penalty + cand.score;
    };

    double best = std::numeric_limits<double>::infinity();
    best = std::min(best, relaxed_score(search.orbit_seed_best, relaxed_penalty));
    best = std::min(best, relaxed_score(search.fuel_seed_best, relaxed_penalty * 1.15));
    best = std::min(best, relaxed_score(search.recovery_seed_best, relaxed_penalty * 1.30));
    best = std::min(best, relaxed_score(search.relaxed_best, relaxed_penalty * 1.45));
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

GlobePt globe_from_eci_r_km(const Vec3& r_km, double earth_rotation_angle_deg) {
    const double rn = std::max(1e-9, norm3(r_km));
    GlobePt gp;
    gp.lat_deg = rad2deg(std::asin(clampd(r_km.z / rn, -1.0, 1.0)));
    gp.lon_deg = wrap_lon_deg(rad2deg(std::atan2(r_km.y, r_km.x)) - earth_rotation_angle_deg);
    gp.alt_km = std::max(0.0, rn - (kRe / 1000.0));
    return gp;
}

GlobePt globe_from_eci_r_m(const Vec3& r_m, double earth_rotation_angle_deg) {
    return globe_from_eci_r_km(r_m / 1000.0, earth_rotation_angle_deg);
}

GlobeSeries traj3d_to_globe(
    const std::wstring& name,
    COLORREF color,
    const std::vector<SimPt3D>& traj,
    double earth_rotation_angle_deg) {
    GlobeSeries gs;
    gs.name = name;
    gs.color = color;
    gs.pts.reserve(traj.size());
    for (const SimPt3D& pt : traj) {
        GlobePt gp = globe_from_eci_r_m(pt.r_m, earth_rotation_angle_deg);
        if (finite_globe_pt(gp)) gs.pts.push_back(gp);
    }
    return gs;
}

bool stage2_insertion_display_anchor(const MissionResult& result, GlobePt& anchor, Vec3& horiz) {
    for (const GlobeSeries& series : result.globe_series) {
        if (series.name != L"Stage2 Insertion" || series.pts.size() < 2) continue;

        size_t last_index = series.pts.size();
        while (last_index > 0 && !finite_globe_pt(series.pts[last_index - 1])) --last_index;
        if (last_index == 0) continue;

        const GlobePt& last = series.pts[last_index - 1];
        const Vec3 rhat = normalize3(ecef_from_geo(last.lat_deg, last.lon_deg, 0.0));
        for (size_t prev_index = last_index - 1; prev_index > 0; --prev_index) {
            const GlobePt& prev = series.pts[prev_index - 1];
            if (!finite_globe_pt(prev)) continue;
            const Vec3 prev_hat = normalize3(ecef_from_geo(prev.lat_deg, prev.lon_deg, 0.0));
            const Vec3 raw{
                rhat.x - prev_hat.x,
                rhat.y - prev_hat.y,
                rhat.z - prev_hat.z,
            };
            const double radial_component = dot3(raw, rhat);
            const Vec3 tangent{
                raw.x - radial_component * rhat.x,
                raw.y - radial_component * rhat.y,
                raw.z - radial_component * rhat.z,
            };
            if (dot3(tangent, tangent) <= 1e-12) continue;
            anchor = last;
            horiz = normalize3(tangent);
            return true;
        }
    }
    return false;
}

void append_post_orbit_series(MissionResult& result, const MissionRequest& request) {
    const Stage2Result& stage2 = result.stage2;
    if (!stage2.orbit_ok) return;

    GlobePt insertion_gp{};
    Vec3 r_vec{};
    Vec3 v_vec{};
    const bool use_state3d = stage2.seco3d.valid;
    if (use_state3d) {
        insertion_gp = globe_from_eci_r_m(stage2.seco3d.r_m, request.earth_rotation_angle_deg);
        r_vec = stage2.seco3d.r_m / 1000.0;
        v_vec = stage2.seco3d.v_mps / 1000.0;
    } else {
        destination_from_course(
            request.lat_deg,
            result.launch_lon_deg,
            result.orbit_target.launch_az_deg,
            std::max(0.0, stage2.seco.theta * (kRe / 1000.0)),
            insertion_gp.lat_deg,
            insertion_gp.lon_deg);
        insertion_gp.alt_km = std::max(0.0, (stage2.seco.r - kRe) / 1000.0);

        Vec3 horiz{};
        if (!stage2_insertion_display_anchor(result, insertion_gp, horiz)) {
            const Vec3 east = normalize3({-std::sin(deg2rad(insertion_gp.lon_deg)), std::cos(deg2rad(insertion_gp.lon_deg)), 0.0});
            const Vec3 north = normalize3({
                -std::sin(deg2rad(insertion_gp.lat_deg)) * std::cos(deg2rad(insertion_gp.lon_deg)),
                -std::sin(deg2rad(insertion_gp.lat_deg)) * std::sin(deg2rad(insertion_gp.lon_deg)),
                std::cos(deg2rad(insertion_gp.lat_deg))});
            const double az_rad = deg2rad(result.orbit_target.launch_az_deg);
            horiz = normalize3({
                north.x * std::cos(az_rad) + east.x * std::sin(az_rad),
                north.y * std::cos(az_rad) + east.y * std::sin(az_rad),
                north.z * std::cos(az_rad) + east.z * std::sin(az_rad),
            });
        }
        insertion_gp.alt_km = std::max(0.0, (stage2.seco.r - kRe) / 1000.0);
        const Vec3 rhat_orbit = normalize3(ecef_from_geo(insertion_gp.lat_deg, insertion_gp.lon_deg, 0.0));
        const double fpa_rad = deg2rad(stage2.orbit.flight_path_deg);
        const Vec3 vdir = normalize3({
            horiz.x * std::cos(fpa_rad) + rhat_orbit.x * std::sin(fpa_rad),
            horiz.y * std::cos(fpa_rad) + rhat_orbit.y * std::sin(fpa_rad),
            horiz.z * std::cos(fpa_rad) + rhat_orbit.z * std::sin(fpa_rad),
        });

        const double rmag_km = stage2.seco.r / 1000.0;
        const double speed_kmps = stage2.orbit.speed_mps / 1000.0;
        r_vec = {rmag_km * rhat_orbit.x, rmag_km * rhat_orbit.y, rmag_km * rhat_orbit.z};
        v_vec = {speed_kmps * vdir.x, speed_kmps * vdir.y, speed_kmps * vdir.z};
    }
    const Vec3 h_vec = cross3(r_vec, v_vec);
    const double h_norm = std::sqrt(dot3(h_vec, h_vec));
    if (h_norm <= 1e-8) return;

    const double rmag_km = std::max(1e-9, norm3(r_vec));
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
    const int n_orbit = 270;
    post_orbit.pts.reserve(static_cast<size_t>(n_orbit + 1));
    for (int i = 0; i <= n_orbit; ++i) {
        if (i == 0) {
            post_orbit.pts.push_back(insertion_gp);
            continue;
        }
        const double nu = nu0 + (3.14159265358979323846 * static_cast<double>(i)) / static_cast<double>(n_orbit);
        const double rmag = p_orb / std::max(1e-8, 1.0 + e_orb * std::cos(nu));
        const Vec3 q{
            rmag * (std::cos(nu) * u_orb.x + std::sin(nu) * w_orb.x),
            rmag * (std::cos(nu) * u_orb.y + std::sin(nu) * w_orb.y),
            rmag * (std::cos(nu) * u_orb.z + std::sin(nu) * w_orb.z),
        };
        const double qn = std::sqrt(dot3(q, q));
        GlobePt gp{};
        if (use_state3d) {
            gp = globe_from_eci_r_km(q, request.earth_rotation_angle_deg);
        } else {
            gp.lat_deg = rad2deg(std::asin(clampd(q.z / std::max(1e-9, qn), -1.0, 1.0)));
            gp.lon_deg = wrap_lon_deg(rad2deg(std::atan2(q.y, q.x)));
            gp.alt_km = std::max(0.0, qn - (kRe / 1000.0));
        }
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

OrbitMetrics orbit_metrics_from_state3d(const StateVector3D& s) {
    OrbitMetrics out;
    if (!s.valid) return out;

    const double r_m = std::max(1.0, norm3(s.r_m));
    const double speed_mps = norm3(s.v_mps);
    const Vec3 r_hat = s.r_m / r_m;
    const double vr_mps = dot3(s.v_mps, r_hat);
    const double vt_mps = std::sqrt(std::max(0.0, speed_mps * speed_mps - vr_mps * vr_mps));
    out.speed_mps = speed_mps;
    out.flight_path_deg = rad2deg(std::atan2(vr_mps, std::max(1e-9, vt_mps)));

    const double r_km = r_m / 1000.0;
    const double v_kmps = speed_mps / 1000.0;
    const Vec3 r_km_vec = s.r_m / 1000.0;
    const Vec3 v_kmps_vec = s.v_mps / 1000.0;
    const Vec3 h_vec = cross3(r_km_vec, v_kmps_vec);
    const double h2 = dot3(h_vec, h_vec);
    const double energy = 0.5 * v_kmps * v_kmps - kMuKm / std::max(1e-9, r_km);
    const double ecc_sq = std::max(0.0, 1.0 + (2.0 * energy * h2) / (kMuKm * kMuKm));
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

    out.launch_az_deg = direct_launch_azimuth_deg(request.lat_deg, request.incl_deg);
    {
        const double lat = deg2rad(request.lat_deg);
        const double lon = deg2rad(wrap_lon_deg(request.launch_lon_deg + request.earth_rotation_angle_deg));
        const double az = deg2rad(out.launch_az_deg);
        out.launch_rhat_eci = normalize3({
            std::cos(lat) * std::cos(lon),
            std::cos(lat) * std::sin(lon),
            std::sin(lat),
        });
        out.launch_east_eci = normalize3({-std::sin(lon), std::cos(lon), 0.0});
        out.launch_north_eci = normalize3({
            -std::sin(lat) * std::cos(lon),
            -std::sin(lat) * std::sin(lon),
            std::cos(lat),
        });
        out.launch_tangent_eci = normalize3(out.launch_north_eci * std::cos(az) + out.launch_east_eci * std::sin(az));
        out.plane_normal_eci = normalize3(cross3(out.launch_rhat_eci, out.launch_tangent_eci));
        if (dot3(out.plane_normal_eci, out.plane_normal_eci) < 1e-10) {
            out.plane_normal_eci = {0.0, 0.0, 1.0};
        }
        out.has_3d_plane = true;
    }

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

double gmst_deg_from_utc_jd(double jd_utc) {
    const double d = jd_utc - 2451545.0;
    const double t = d / 36525.0;
    return wrap360_deg(
        280.46061837 +
        360.98564736629 * d +
        0.000387933 * t * t -
        (t * t * t) / 38710000.0);
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
    if (std::isfinite(out.launch_epoch_utc_jd)) {
        out.earth_rotation_angle_deg = gmst_deg_from_utc_jd(out.launch_epoch_utc_jd);
    } else {
        out.earth_rotation_angle_deg = wrap360_deg(out.earth_rotation_angle_deg);
    }
    if (std::isfinite(out.target_raan_deg)) out.target_raan_deg = wrap360_deg(out.target_raan_deg);
    out.launch_window_half_width_min = clampd(out.launch_window_half_width_min, 1.0, 720.0);
    return out;
}

MissionResult solve_mission(const MissionRequest& request_in, SolveControl control) {
    MissionProfile& profile = mission_profile();
    const bool profile_enabled = mission_profile_requested();
    profile.reset(profile_enabled);
    const ProfileClock::time_point solve_profile_start = ProfileClock::now();

    const MissionRequest request = sanitize_request(request_in);
    const bool recovery_required = !control.ignore_recovery;
    MissionResult result;
    result.orbit_target = build_orbit_target(request);
    result.launch_lat_deg = request.lat_deg;
    result.launch_lon_deg = request.launch_lon_deg;
    result.launch_epoch_utc_jd = request.launch_epoch_utc_jd;

    LvdOptions lvd_options;
    lvd_options.allow_full_burn = !recovery_required || control.force_stage1_burnout;
    lvd_options.force_stage1_burnout = control.force_stage1_burnout;
    lvd_options.cancel_requested = control.cancel_requested;
    lvd_options.max_mission_design_evals = control.force_stage1_burnout ? 6 : 18;
    lvd_options.mission_score = [&](const LvdResult& candidate_lvd) {
        if (candidate_lvd.state_samples.empty() || !candidate_lvd.stage1.converged) {
            return std::numeric_limits<double>::infinity();
        }
        std::vector<double> sep_times = build_lvd_score_sep_times(request, candidate_lvd);
        if (sep_times.empty()) return std::numeric_limits<double>::infinity();
        SolveControl lvd_score_control = control;
        lvd_score_control.worker_count = 1;
        SepSearchResult search;
        {
            ScopedProfile search_scope(profile.lvd_mission_search, profile.enabled);
            search = search_lvd_sep_samples(
                request,
                result.orbit_target,
                candidate_lvd,
                sep_times,
                lvd_score_control,
                nullptr);
        }
        return score_lvd_search_result(search, recovery_required);
    };
    LvdResult lvd;
    {
        ScopedProfile lvd_scope(profile.lvd_total, profile.enabled);
        lvd = solve_stage1_lvd(request, result.orbit_target, lvd_options);
    }

    result.lvd_time_series = lvd.time_series;
    result.lvd_events = lvd.events;
    result.launch_window_samples = lvd.launch_window_samples;
    result.lvd_launch_offset_s = lvd.launch_offset_s;
    result.lvd_earth_rotation_angle_deg = lvd.earth_rotation_angle_deg;
    result.lvd_launch_raan_deg = lvd.launch_raan_deg;
    result.lvd_target_raan_deg = lvd.target_raan_deg;
    result.lvd_plane_error_deg = lvd.plane_error_deg;

    std::vector<SeparationCandidate> sep_sweep_samples;
    SeparationCandidate best;
    bool best_is_accepted = false;
    auto timed_final_sep_search = [&](
        const std::vector<double>& sep_times,
        std::vector<SeparationCandidate>* sample_out) {
        ScopedProfile search_scope(profile.final_sep_search, profile.enabled);
        return search_lvd_sep_samples(
            request,
            result.orbit_target,
            lvd,
            sep_times,
            control,
            sample_out);
    };
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

    const double sep_delay_s = clampd(request.s1_sep_delay_s, 1.5, 6.0);
    const double sep_hi = std::max(sep_delay_s, lvd.stage1.sep_s);
    const double sep_lo = std::min(
        sep_hi,
        std::max(sep_delay_s + 60.0, sep_delay_s + 0.55 * std::max(1.0, lvd.stage1.meco_s)));

    if (control.force_stage1_burnout) {
        const std::vector<double> burnout_sep_times{sep_hi};
        SepSearchResult burnout_search = timed_final_sep_search(burnout_sep_times, &sep_sweep_samples);
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
        SepSearchResult coarse_search = timed_final_sep_search(coarse_sep_times, &sep_sweep_samples);
        promote_best_from_search(coarse_search, true);
    } else {
        std::vector<double> coarse_sep_times;
        coarse_sep_times.reserve(9);
        for (int i = 0; i <= 8; ++i) {
            const double u = static_cast<double>(i) / 8.0;
            coarse_sep_times.push_back(lerpd(sep_lo, sep_hi, u));
        }
        SepSearchResult coarse_search = timed_final_sep_search(coarse_sep_times, &sep_sweep_samples);
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
            SepSearchResult fallback_search = timed_final_sep_search(fallback_sep_time, &sep_sweep_samples);
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
            SepSearchResult refined = timed_final_sep_search(refine_sep_times, &sep_sweep_samples);
            promote_best_from_search(refined, false);
        }
    }

    if (!std::isfinite(best.score)) {
        result.status = L"MARGINAL / NOT FEASIBLE";
        result.payload_search_ok = false;
        result.ok = false;
        result.lines.push_back(L"[Status] " + result.status);
        append_orbit_target_report(result.lines, request, result.orbit_target);
        result.lines.push_back(L"[Reserve Fuel] Stage1=N/A kg, Stage2=N/A kg");
        result.lines.push_back(L"[Reserve Objective] maximize Stage2 remaining propellant = N/A");
        if (recovery_required) {
            result.lines.push_back(L"[LVD] Stage1 LVD did not produce a finite SEP candidate for Stage2 insertion and recovery evaluation.");
        } else {
            result.lines.push_back(L"[LVD] Stage1 LVD did not produce a finite SEP candidate; recovery constraints were ignored for this mode.");
        }
        if (profile.enabled) {
            profile.solve_total.add(ProfileClock::now() - solve_profile_start);
            print_mission_profile(profile);
        }
        return result;
    }

    {
        ScopedProfile report_scope(profile.report_build, profile.enabled);

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
        append_orbit_target_report(result.lines, request, result.orbit_target);
        if (!recovery_required) {
            result.lines.push_back(L"[Mode] No recovery; Stage1 burns to depletion before separation.");
        }
        result.lines.push_back(
            L"[LVD Window] offset=" + fnum(result.lvd_launch_offset_s, 1) +
            L" s, earth_rotation=" + fnum(result.lvd_earth_rotation_angle_deg, 3) +
            L" deg, launch_RAAN=" + fnum(result.lvd_launch_raan_deg, 3) +
            L" deg, target_RAAN=" + fnum(result.lvd_target_raan_deg, 3) +
            L" deg, plane_err=" + fnum(result.lvd_plane_error_deg, 4) + L" deg");
        result.lines.push_back(
            L"[Stage1 LVD Target] sep=" + fnum(best.sep_time_s, 1) +
            L" s, tgt_alt=" + fnum(best.sep_alt_target_km, 1) +
            L" km, tgt_speed=" + fnum(best.sep_speed_target_mps, 0) +
            L" m/s, tgt_gamma=" + fnum(best.sep_gamma_target_deg, 1) + L" deg");
        result.lines.push_back(
            L"[Stage1 LVD] MECO=" + fnum(result.stage1.meco_s, 1) +
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
        ascent.name = L"Stage1 LVD Ascent";
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
            if (series.name == L"Stage1 LVD Ascent" && !result.stage1.traj3d.empty()) {
                result.globe_series.push_back(traj3d_to_globe(series.name, series.color, result.stage1.traj3d, request.earth_rotation_angle_deg));
            } else if (series.name == L"Stage2 Insertion" && !result.stage2.traj3d.empty()) {
                result.globe_series.push_back(traj3d_to_globe(series.name, series.color, result.stage2.traj3d, request.earth_rotation_angle_deg));
            } else {
                result.globe_series.push_back(profile_to_globe(series, request, result.orbit_target.launch_az_deg, result.launch_lon_deg));
            }
        }
        append_post_orbit_series(result, request);
    }
    if (profile.enabled) {
        profile.solve_total.add(ProfileClock::now() - solve_profile_start);
        print_mission_profile(profile);
    }

    return result;
}

}  // namespace falcon9
