#include "planner_recovery.hpp"
#include "GFOLD_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace falcon9 {

namespace {

struct LocalState {
    double x = 0.0;
    double z = 0.0;
    double vx = 0.0;
    double vz = 0.0;
    double m = 0.0;
};

struct LandingPhaseResult {
    bool success = false;
    double touchdown_time_s = 0.0;
    double touchdown_downrange_km = 0.0;
    double touchdown_speed_mps = 0.0;
    double prop_used_kg = 0.0;
    double terminal_mass_kg = std::numeric_limits<double>::quiet_NaN();
    std::vector<SimPt> traj;
};

constexpr double kGfold2dVelocityLimitMps = 300.0;
constexpr std::array<double, 10> kGfold2dTfCandidatesS = {8.0, 12.0, 16.0, 20.0, 25.0, 30.0, 40.0, 55.0, 70.0, 90.0};

std::vector<double> select_landing_tf_candidates(const MissionRequest& request, const LocalState& s) {
    const double stage_thrust_N = request.s1_thrust_kN * 1000.0;
    const double thrust_max_N = stage_thrust_N / 3.0;
    const double g = grav(std::max(0.0, s.z));
    const double thrust_acc_max = thrust_max_N / std::max(1.0, s.m);
    const double usable_decel = std::max(1.0, thrust_acc_max - g);
    const double speed = std::hypot(s.vx, s.vz);
    const double t_brake = speed / usable_decel;
    const double t_fall = std::sqrt(2.0 * std::max(0.0, s.z) / std::max(1.0, g));
    const double t_est = clampd(t_brake + 0.35 * t_fall, kGfold2dTfCandidatesS.front(), kGfold2dTfCandidatesS.back());

    int center = 0;
    double best_err = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(kGfold2dTfCandidatesS.size()); ++i) {
        const double err = std::abs(kGfold2dTfCandidatesS[static_cast<std::size_t>(i)] - t_est);
        if (err < best_err) {
            best_err = err;
            center = i;
        }
    }

    std::vector<double> out;
    out.reserve(4);
    auto add_idx = [&](int idx) {
        if (idx < 0 || idx >= static_cast<int>(kGfold2dTfCandidatesS.size())) return;
        const double tf = kGfold2dTfCandidatesS[static_cast<std::size_t>(idx)];
        if (std::find(out.begin(), out.end(), tf) == out.end()) out.push_back(tf);
    };
    add_idx(center);
    add_idx(center - 1);
    add_idx(center + 1);
    if (s.z < 500.0) add_idx(0);
    return out;
}

LocalState local_from_stage1_sep(const MissionRequest& request, const Stage1Result& stage1) {
    LocalState out;
    out.x = stage1.sep.theta * kRe;
    out.z = std::max(0.0, stage1.sep.r - kRe);
    out.vx = stage1.sep.vt;
    out.vz = stage1.sep.vr;
    out.m = request.s1_dry_kg + std::max(0.0, stage1.rem_prop);
    return out;
}

void push_traj(std::vector<SimPt>& traj, double t_global, const LocalState& s) {
    if (traj.empty() || t_global - traj.back().t >= 1.0) {
        traj.push_back({t_global, s.x / 1000.0, std::max(0.0, s.z / 1000.0)});
    }
}

// Simulate the landing burn from `ignition_state` until touchdown or until the
// allowed propellant `prop_budget_kg` is exhausted.  IMPORTANT: the booster's
// initial mass is taken straight from `ignition_state.m` (which reflects the
// real dry+rem_prop after the coast).  The previous version overwrote the
// initial mass with `dry + prop_budget`, which made the binary search probe
// physically inconsistent (lighter rocket → falsely lower fuel need) and was
// the dominant reason the recovery solver kept declaring scenarios infeasible.
[[maybe_unused]] LandingPhaseResult simulate_landing_phase_legacy(
    const MissionRequest& request,
    const LocalState& ignition_state,
    double ignition_time_s,
    double prop_budget_kg,
    bool record_traj) {
    LandingPhaseResult out;
    LocalState s = ignition_state;
    // The booster carries dry + rem_prop in reality.  prop_budget_kg only
    // limits how much of that we are *allowed* to consume during this probe.
    if (s.m <= request.s1_dry_kg + 1.0) {
        s.m = request.s1_dry_kg + std::max(0.0, prop_budget_kg);
    }

    // Landing burn should be constrained to one Merlin, not all 9 first-stage engines.
    // request.s1_thrust_kN is the full stage-1 sea-level thrust, so use 1/9 here.
    const double thrust_max = (request.s1_thrust_kN * 1000.0) / 9.0;
    const double throttle_min = 0.15;
    // Terminal phase hover authority: below this altitude the guidance is
    // allowed to throttle all the way down to match real single-engine hover
    // thrust.  Without this, the aggregate-Stage-1 minimum throttle (0.15)
    // forces an effective 35+ m/s² upward acceleration a few meters above
    // the ground, causing the closed-loop controller to oscillate and leave
    // a stubborn ≈20 m/s residual at touchdown even when propellant is
    // abundant.
    const double terminal_throttle_min = 0.04;
    const double terminal_alt_m = 150.0;
    const double mdot_nom = thrust_max / std::max(1e-6, request.s1_isp_s * kG0);
    const double cda = 14.0;
    const double burn_dt = 0.05;
    const double max_burn_t = 240.0;
    double remaining_prop = std::max(0.0, prop_budget_kg);
    double burn_elapsed = 0.0;

    if (record_traj) {
        out.traj.reserve(512);
        out.traj.push_back({ignition_time_s, s.x / 1000.0, s.z / 1000.0});
    }

    while (burn_elapsed < max_burn_t && s.z > 0.0 && remaining_prop > 1e-6) {
        const double speed = std::hypot(s.vx, s.vz);
        const double g = grav(s.z);
        const double r = rho(s.z);
        const double drag = 0.5 * r * speed * speed * cda;

        const double thrust_acc_max = thrust_max / std::max(1.0, s.m);
        const double usable_decel = std::max(1.0, thrust_acc_max - g);

        // Time-to-go heuristic.  The previous code clamped tgo to [1.2, 18] s
        // which was far too tight: from a 50 km, 2.5 km/s state the position
        // feedback term `2*(0 - z - vz*tgo)/tgo^2` then dwarfed the velocity
        // term and (after re-adding gravity) commanded thrust DOWNWARD,
        // accelerating the descent instead of arresting it.  The new bound is
        // generous enough that the position term stays well-conditioned at
        // real ignition altitudes.
        const double tgo = clampd(speed / usable_decel + std::sqrt(2.0 * std::max(0.0, s.z) / usable_decel),
                                  2.0, 60.0);

        // Acceleration-feedback bounds raised to ~8 g (Falcon 9 booster can
        // pull more during a single-engine landing burn).  The previous ±18 /
        // ±20 m/s² caps held the closed-loop gain so low that the solver
        // never reached a successful touchdown except for nearly trivial
        // entry states.
        const double a_lim = 80.0;
        const double ax_fb = clampd((0.0 - s.vx) / tgo, -a_lim, a_lim);
        const double az_fb = clampd(
            (0.0 - s.vz) / tgo + 2.0 * (0.0 - s.z - s.vz * tgo) / std::max(1.0, tgo * tgo),
            -a_lim,
            a_lim);

        double thrust_ax_cmd = ax_fb;
        double thrust_az_cmd = az_fb + g;
        if (speed > 1e-4) {
            const double invm = 1.0 / std::max(1.0, s.m);
            thrust_ax_cmd += (drag * invm) * (s.vx / speed);
            thrust_az_cmd += (drag * invm) * (s.vz / speed);
        }

        // Far above the ground (or while still moving fast) the safest action
        // is to point the thrust antiparallel to velocity – this guarantees a
        // monotonic decrease in kinetic energy and prevents the rare cases
        // where the linearised ZEM/ZEV law would otherwise dip the thrust
        // vector below the horizon.
        if (s.z > 1500.0 && speed > 60.0) {
            const double vmag = std::max(1e-6, speed);
            const double vhat_x = s.vx / vmag;
            const double vhat_z = s.vz / vmag;
            const double a_brake = std::min(usable_decel, a_lim);
            const double brake_x = -vhat_x * a_brake;
            const double brake_z = -vhat_z * a_brake + g; // cancel gravity too
            // Blend: pure brake high up, ZEM/ZEV nearer the ground.
            const double blend = clampd((s.z - 1500.0) / 4000.0, 0.0, 1.0);
            thrust_ax_cmd = lerpd(thrust_ax_cmd, brake_x, blend);
            thrust_az_cmd = lerpd(thrust_az_cmd, brake_z, blend);
        }

        // Hard guarantee: never command a downward thrust component while we
        // are still at altitude – the engine must always at least support the
        // booster's own weight.
        if (s.z < 60.0 && thrust_az_cmd < 0.1) thrust_az_cmd = 0.1;
        if (s.z >= 60.0 && thrust_az_cmd < 0.0) thrust_az_cmd = 0.0;

        const double thrust_cmd_mag = std::sqrt(thrust_ax_cmd * thrust_ax_cmd + thrust_az_cmd * thrust_az_cmd);
        const double active_throttle_min = (s.z < terminal_alt_m) ? terminal_throttle_min : throttle_min;
        const double throttle_cmd = clampd(thrust_cmd_mag / std::max(1e-6, thrust_acc_max), active_throttle_min, 1.0);
        const double dm = std::min(mdot_nom * throttle_cmd * burn_dt, remaining_prop);
        const double mdot = dm / std::max(1e-9, burn_dt);
        const double thrust_now = mdot * request.s1_isp_s * kG0;
        const double invm = 1.0 / std::max(1.0, s.m);
        const double cmd_norm = std::max(1e-6, thrust_cmd_mag);

        double ax = (thrust_now * (thrust_ax_cmd / cmd_norm)) * invm;
        double az = (thrust_now * (thrust_az_cmd / cmd_norm)) * invm - g;
        if (speed > 1e-6) {
            ax -= (drag * invm) * (s.vx / speed);
            az -= (drag * invm) * (s.vz / speed);
        }

        s.vx += ax * burn_dt;
        s.vz += az * burn_dt;
        s.x += s.vx * burn_dt;
        s.z += s.vz * burn_dt;
        s.m -= dm;
        remaining_prop -= dm;
        burn_elapsed += burn_dt;
        if (record_traj) push_traj(out.traj, ignition_time_s + burn_elapsed, s);

        // Early-exit guards so we don't loiter in the low-altitude terminal
        // phase once the booster is essentially stopped above the pad – this
        // bounds the recovery simulation runtime even when the guidance
        // converges to a near-hover command on the last few meters.
        if (s.z < 10.0 && std::hypot(s.vx, s.vz) < 1.5) {
            s.z = 0.0;
            break;
        }
        if (s.z <= 0.0) break;
    }

    // If the burn exhausted propellant while the vehicle is already near the
    // ground with a low residual velocity, finish the descent by coasting.
    // Without this the solver would flag otherwise-landable profiles as
    // infeasible because the simulation terminated a few meters high.
    if (s.z > 0.0 && remaining_prop <= 1e-6) {
        const double vx0 = s.vx;
        const double vz0 = s.vz;
        const double speed0 = std::hypot(vx0, vz0);
        if (s.z < 300.0 && speed0 < 25.0) {
            const double g = grav(std::max(0.0, s.z));
            // Solve: s.z + vz*t + 0.5*(-g)*t^2 = 0  =>  0.5*g*t^2 - vz*t - s.z = 0.
            const double a = 0.5 * g;
            const double b = -vz0;
            const double c = -s.z;
            const double disc = std::max(0.0, b * b - 4.0 * a * c);
            const double t_fall = (-b + std::sqrt(disc)) / std::max(1e-6, 2.0 * a);
            if (std::isfinite(t_fall) && t_fall > 0.0 && t_fall < 30.0) {
                s.x += vx0 * t_fall;
                s.vz = vz0 - g * t_fall;
                s.vx = vx0;
                s.z = 0.0;
                burn_elapsed += t_fall;
                if (record_traj) push_traj(out.traj, ignition_time_s + burn_elapsed, s);
            }
        }
    }

    out.touchdown_time_s = ignition_time_s + burn_elapsed;
    out.touchdown_downrange_km = s.x / 1000.0;
    out.touchdown_speed_mps = std::hypot(s.vx, s.vz);
    out.prop_used_kg = std::max(0.0, prop_budget_kg - remaining_prop);
    out.success =
        std::isfinite(out.touchdown_time_s) &&
        std::isfinite(out.touchdown_downrange_km) &&
        std::isfinite(out.touchdown_speed_mps) &&
        s.z <= 5.0 &&
        out.touchdown_speed_mps <= 12.0;
    return out;
}

LandingPhaseResult simulate_landing_phase(
    const MissionRequest& request,
    const LocalState& ignition_state,
    double ignition_time_s,
    double prop_budget_kg,
    bool record_traj) {
    LandingPhaseResult out;
    LocalState s = ignition_state;
    if (s.m <= request.s1_dry_kg + 1.0) {
        s.m = request.s1_dry_kg + std::max(0.0, prop_budget_kg);
    }

    const double speed = std::hypot(s.vx, s.vz);
    out.touchdown_time_s = ignition_time_s;
    out.touchdown_downrange_km = s.x / 1000.0;
    out.touchdown_speed_mps = speed;
    out.terminal_mass_kg = s.m;

    if (!std::isfinite(speed) || speed > kGfold2dVelocityLimitMps || s.z <= 0.0 ||
        s.m <= request.s1_dry_kg + 1.0 || prop_budget_kg <= 0.0) {
        return out;
    }

    const double stage_thrust_N = request.s1_thrust_kN * 1000.0;
    const double thrust_max_N = stage_thrust_N / 3.0;
    const double thrust_min_N = (stage_thrust_N / 9.0) * 0.15;
    if (thrust_max_N <= 0.0 || thrust_min_N <= 0.0 || thrust_min_N > thrust_max_N) {
        return out;
    }

    GFOLDSolution best_solution;
    double best_terminal_mass = std::numeric_limits<double>::quiet_NaN();
    double best_prop_used = std::numeric_limits<double>::infinity();
    double best_speed = std::numeric_limits<double>::infinity();
    double best_tf = std::numeric_limits<double>::quiet_NaN();
    bool have_best = false;

    for (const double tf : select_landing_tf_candidates(request, s)) {
        GFOLDConfig cfg;
        cfg.steps = 50;
        cfg.solver_n = 50;
        cfg.tf = tf;
        cfg.elapsed_time = ignition_time_s;
        cfg.g0 = grav(std::max(0.0, s.z));
        cfg.Isp = request.s1_isp_s;
        cfg.T_max = thrust_max_N;
        cfg.throttle_min = thrust_min_N / thrust_max_N;
        cfg.throttle_max = 1.0;
        cfg.m0 = s.m;
        cfg.r0[0] = std::max(0.0, s.z);
        cfg.r0[1] = 0.0;
        cfg.r0[2] = 0.0;
        cfg.v0[0] = s.vz;
        cfg.v0[1] = s.vx;
        cfg.v0[2] = 0.0;
        cfg.glide_slope_deg = 30.0;
        cfg.max_angle_deg = 45.0;

        GFOLDSolution sol;
        GFOLDSolverInfo info;
        double terminal_mass = std::numeric_limits<double>::quiet_NaN();
        if (!solve_gfold_p4_n50_2d_free_x(cfg, sol, &info, nullptr, &terminal_mass)) {
            continue;
        }
        if (!std::isfinite(terminal_mass) || terminal_mass < request.s1_dry_kg - 1e-3) {
            continue;
        }
        if (sol.steps <= 0 || sol.vx.size() != static_cast<std::size_t>(sol.steps) ||
            sol.vy.size() != static_cast<std::size_t>(sol.steps)) {
            continue;
        }

        const std::size_t last = static_cast<std::size_t>(sol.steps - 1);
        const double terminal_speed = std::hypot(sol.vx[last], sol.vy[last]);
        const double prop_used = std::max(0.0, s.m - terminal_mass);
        if (!std::isfinite(terminal_speed) || !std::isfinite(prop_used) ||
            prop_used > prop_budget_kg + 1e-3 || terminal_speed > 12.0) {
            continue;
        }

        const bool better =
            !have_best ||
            prop_used < best_prop_used - 1e-6 ||
            (std::abs(prop_used - best_prop_used) <= 1e-6 &&
             (terminal_speed < best_speed - 1e-6 ||
              (std::abs(terminal_speed - best_speed) <= 1e-6 && tf < best_tf)));
        if (better) {
            have_best = true;
            best_solution = std::move(sol);
            best_terminal_mass = terminal_mass;
            best_prop_used = prop_used;
            best_speed = terminal_speed;
            best_tf = tf;
        }
    }

    if (!have_best) return out;

    const std::size_t last = static_cast<std::size_t>(best_solution.steps - 1);
    const double terminal_downrange_m = s.x + best_solution.ry[last];

    out.success = true;
    out.touchdown_time_s = ignition_time_s + best_tf;
    out.touchdown_downrange_km = terminal_downrange_m / 1000.0;
    out.touchdown_speed_mps = best_speed;
    out.prop_used_kg = best_prop_used;
    out.terminal_mass_kg = best_terminal_mass;

    if (record_traj) {
        out.traj.reserve(static_cast<std::size_t>(best_solution.steps));
        for (int i = 0; i < best_solution.steps; ++i) {
            const std::size_t idx = static_cast<std::size_t>(i);
            out.traj.push_back({
                best_solution.t[idx],
                (s.x + best_solution.ry[idx]) / 1000.0,
                std::max(0.0, best_solution.rx[idx] / 1000.0),
            });
        }
    }
    return out;
}

}  // namespace

RecoveryResult simulate_stage1_recovery(
    const MissionRequest& request,
    const Stage1Result& stage1,
    double launch_az_deg) {
    RecoveryResult out;
    LocalState s = local_from_stage1_sep(request, stage1);
    auto estimate_prop_need_from_state = [&](const LocalState& q) {
        const double speed = std::hypot(q.vx, q.vz);
        const double g_now = grav(std::max(0.0, q.z));
        const double mean_mass = std::max(request.s1_dry_kg + 0.5 * std::max(0.0, stage1.rem_prop), request.s1_dry_kg + 1.0);
        const double thrust_acc = (request.s1_thrust_kN * 1000.0) / mean_mass;
        const double usable_decel = std::max(0.5, thrust_acc - g_now);
        const double t_stop_est = speed / usable_decel;
        const double gravity_loss = g_now * t_stop_est;
        const double dv_need = speed + gravity_loss + 40.0;
        return prop_for_dv(
            request.s1_dry_kg + std::max(0.0, stage1.rem_prop),
            dv_need,
            request.s1_isp_s);
    };

    const double cda = 14.0;
    const double coast_dt = 0.20;
    const double max_t = 2000.0;

    out.coast_traj.reserve(512);
    out.landing_traj.reserve(512);
    out.coast_traj.push_back({stage1.sep_s, s.x / 1000.0, s.z / 1000.0});

    double t_local = 0.0;
    const double landing_prop_before = std::max(0.0, stage1.rem_prop);
    bool found_feasible_ignition = false;
    bool have_any_ignition_eval = false;
    LocalState best_ign_state = s;
    double best_ign_time_s = stage1.sep_s;
    double best_required_prop = std::numeric_limits<double>::infinity();
    LandingPhaseResult best_phase{};
    double check_accum_s = 0.0;

    while (t_local < max_t && s.z > 0.0) {
        if (s.z > 5.0) {
            check_accum_s += coast_dt;
            if (check_accum_s >= 1.0) {
                check_accum_s = 0.0;
                const double ignition_time_s = stage1.sep_s + t_local;
                const double rough_required_prop = estimate_prop_need_from_state(s);
                LandingPhaseResult full_budget_try;
                if (rough_required_prop > landing_prop_before + 500.0) {
                    full_budget_try.touchdown_time_s = ignition_time_s;
                    full_budget_try.touchdown_downrange_km = s.x / 1000.0;
                    full_budget_try.touchdown_speed_mps = std::hypot(s.vx, s.vz);
                    full_budget_try.prop_used_kg = 0.0;
                } else {
                    full_budget_try = simulate_landing_phase(request, s, ignition_time_s, landing_prop_before, false);
                }
                // The GFOLD landing solve reports the terminal mass directly,
                // so a successful run's consumed propellant is the landing
                // fuel cost for this ignition state.
                double required_prop = full_budget_try.success
                    ? std::max(0.0, full_budget_try.prop_used_kg)
                    : std::max(landing_prop_before, rough_required_prop);
                if (!full_budget_try.success) {
                    const double residual_prop_need =
                        prop_for_dv(
                            std::max(request.s1_dry_kg + std::max(0.0, landing_prop_before - full_budget_try.prop_used_kg),
                                     request.s1_dry_kg + 1.0),
                            2.0 * std::max(0.0, full_budget_try.touchdown_speed_mps - 10.0) + 80.0,
                            request.s1_isp_s);
                    required_prop = std::max(required_prop, landing_prop_before + residual_prop_need);
                }

                const bool better =
                    !have_any_ignition_eval ||
                    (full_budget_try.success && !found_feasible_ignition) ||
                    (full_budget_try.success == found_feasible_ignition &&
                     (required_prop < best_required_prop - 1e-6 ||
                      (std::abs(required_prop - best_required_prop) <= 1e-6 &&
                       full_budget_try.touchdown_speed_mps < best_phase.touchdown_speed_mps - 1e-6)));
                if (better) {
                    have_any_ignition_eval = true;
                    found_feasible_ignition = full_budget_try.success;
                    best_required_prop = required_prop;
                    best_ign_state = s;
                    best_ign_time_s = ignition_time_s;
                    best_phase = full_budget_try;
                }
                // Once we already have a feasible ignition point, stop trying
                // later (deeper, faster) candidates which only get worse.
                // This collapses the search effort on a typical recovery
                // trajectory from ~60 probes to ~10–20.
                if (found_feasible_ignition && !full_budget_try.success) {
                    break;
                }
            }
        }

        const double speed = std::hypot(s.vx, s.vz);
        const double g = grav(s.z);
        const double r = rho(s.z);
        const double drag = 0.5 * r * speed * speed * cda;
        double ax = 0.0;
        double az = -g;
        if (speed > 1e-6) {
            const double invm = 1.0 / std::max(1.0, s.m);
            ax -= (drag * invm) * (s.vx / speed);
            az -= (drag * invm) * (s.vz / speed);
        }

        s.vx += ax * coast_dt;
        s.vz += az * coast_dt;
        s.x += s.vx * coast_dt;
        s.z += s.vz * coast_dt;
        t_local += coast_dt;
        push_traj(out.coast_traj, stage1.sep_s + t_local, s);

        if (s.z <= 0.0) {
            if (have_any_ignition_eval) break;
            out.touchdown_time_s = stage1.sep_s + t_local;
            out.touchdown_downrange_km = s.x / 1000.0;
            out.touchdown_speed_mps = std::hypot(s.vx, s.vz);
            destination_from_course(
                request.lat_deg,
                request.launch_lon_deg,
                launch_az_deg,
                out.touchdown_downrange_km,
                out.touchdown_lat_deg,
                out.touchdown_lon_deg);
            out.landing_prop_kg = estimate_prop_need_from_state(s);
            out.margin_kg = landing_prop_before - out.landing_prop_kg;
            out.feasible = false;
            out.converged = false;
            return out;
        }
    }

    if (!have_any_ignition_eval) {
        out.touchdown_time_s = stage1.sep_s + t_local;
        out.touchdown_downrange_km = s.x / 1000.0;
        out.touchdown_speed_mps = std::hypot(s.vx, s.vz);
        destination_from_course(
            request.lat_deg,
            request.launch_lon_deg,
            launch_az_deg,
            out.touchdown_downrange_km,
            out.touchdown_lat_deg,
            out.touchdown_lon_deg);
        out.landing_prop_kg = estimate_prop_need_from_state(s);
        out.margin_kg = landing_prop_before - out.landing_prop_kg;
        out.feasible = false;
        out.converged = false;
        return out;
    }

    if (!found_feasible_ignition) {
        out.touchdown_time_s = best_phase.touchdown_time_s > 0.0
            ? best_phase.touchdown_time_s
            : stage1.sep_s + t_local;
        out.touchdown_downrange_km = std::isfinite(best_phase.touchdown_downrange_km)
            ? best_phase.touchdown_downrange_km
            : best_ign_state.x / 1000.0;
        out.touchdown_speed_mps = std::isfinite(best_phase.touchdown_speed_mps)
            ? best_phase.touchdown_speed_mps
            : std::hypot(best_ign_state.vx, best_ign_state.vz);
        destination_from_course(
            request.lat_deg,
            request.launch_lon_deg,
            launch_az_deg,
            out.touchdown_downrange_km,
            out.touchdown_lat_deg,
            out.touchdown_lon_deg);
        out.landing_prop_kg = std::max(landing_prop_before + 1.0, estimate_prop_need_from_state(best_ign_state));
        if (std::isfinite(best_required_prop)) {
            out.landing_prop_kg = std::max(out.landing_prop_kg, best_required_prop);
        }
        out.margin_kg = landing_prop_before - out.landing_prop_kg;
        out.feasible = false;
        out.converged = false;
        return out;
    }

    out.landing_ignition_time_s = best_ign_time_s;
    const LandingPhaseResult actual =
        simulate_landing_phase(request, best_ign_state, out.landing_ignition_time_s, landing_prop_before, true);
    out.landing_traj = actual.traj;
    out.touchdown_time_s = actual.touchdown_time_s;
    out.touchdown_downrange_km = actual.touchdown_downrange_km;
    out.touchdown_speed_mps = actual.touchdown_speed_mps;
    destination_from_course(
        request.lat_deg,
        request.launch_lon_deg,
        launch_az_deg,
        out.touchdown_downrange_km,
        out.touchdown_lat_deg,
        out.touchdown_lon_deg);

    double final_prop_kg = std::max(0.0, actual.prop_used_kg);
    if (actual.success && std::isfinite(actual.terminal_mass_kg)) {
        out.landing_prop_kg = final_prop_kg;
        out.margin_kg = actual.terminal_mass_kg - request.s1_dry_kg;
    } else {
        const double residual_prop_need =
            prop_for_dv(
                std::max(request.s1_dry_kg + std::max(0.0, landing_prop_before - actual.prop_used_kg),
                         request.s1_dry_kg + 1.0),
                2.0 * std::max(0.0, actual.touchdown_speed_mps - 10.0) + 60.0,
                request.s1_isp_s);
        final_prop_kg = landing_prop_before + residual_prop_need;
        out.landing_prop_kg = final_prop_kg;
        out.margin_kg = landing_prop_before - out.landing_prop_kg;
    }
    out.converged =
        actual.success &&
        std::isfinite(out.touchdown_time_s) &&
        std::isfinite(out.touchdown_downrange_km) &&
        std::isfinite(out.touchdown_speed_mps) &&
        std::isfinite(out.landing_prop_kg);
    out.feasible = actual.success && out.margin_kg >= -1e-6;
    return out;
}

}  // namespace falcon9
