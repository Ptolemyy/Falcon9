#include "planner_lvd.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace falcon9 {

namespace {

constexpr double kAcceptedMissionScoreCutoff = 1.0e8;

struct FlatState {
    Vec3 r{kRe, 0.0, 0.0};
    Vec3 v{0.0, 0.0, 0.0};
    double m = 0.0;
};

struct LocalFrame3D {
    Vec3 rhat{1.0, 0.0, 0.0};
    Vec3 that{0.0, 1.0, 0.0};
    Vec3 chat{0.0, 0.0, 1.0};
};

struct LvdDesign {
    double burn_s = 0.0;
    double sep_alt_km = 0.0;
    double sep_speed_mps = 0.0;
    double sep_gamma_deg = 0.0;
    double pitch_bias_early_deg = 0.0;
    double pitch_bias_mid_deg = 0.0;
    double pitch_bias_late_deg = 0.0;
    double pitch_bias_terminal_deg = 0.0;
    double pitch_min_bias_deg = 0.0;
    double q_soft_bias_kpa = 0.0;
    double throttle_floor_mid_bias = 0.0;
    double throttle_floor_late_bias = 0.0;
    double q_bucket_gain = 0.020;
    double pitch_rate_scale = 1.0;
    double terminal_gain_scale = 1.0;
};

struct LvdSample {
    double t_s = 0.0;
    double downrange_km = 0.0;
    double alt_km = 0.0;
    double speed_mps = 0.0;
    double gamma_deg = 0.0;
    double q_kpa = 0.0;
    double throttle = 0.0;
    double mass_kg = 0.0;
    StateVector3D state3d;
};

struct CandidateSim {
    Stage1Result stage1;
    LvdDesign design;
    std::vector<LvdSample> samples;
    double score = std::numeric_limits<double>::infinity();
};

struct MissionScoredSim {
    CandidateSim sim;
    double mission_score = std::numeric_limits<double>::infinity();
};

bool cancellation_requested(const LvdOptions& options) {
    return options.cancel_requested && options.cancel_requested->load(std::memory_order_relaxed);
}

double launch_raan_deg_for_rotation(
    const MissionRequest& request,
    const OrbitTarget& orbit_target,
    double earth_rotation_angle_deg) {
    const double lat = deg2rad(request.lat_deg);
    const double lon = deg2rad(wrap_lon_deg(request.launch_lon_deg + earth_rotation_angle_deg));
    const double az = deg2rad(orbit_target.launch_az_deg);

    const Vec3 rhat{
        std::cos(lat) * std::cos(lon),
        std::cos(lat) * std::sin(lon),
        std::sin(lat),
    };
    const Vec3 east = normalize3({-std::sin(lon), std::cos(lon), 0.0});
    const Vec3 north = normalize3({
        -std::sin(lat) * std::cos(lon),
        -std::sin(lat) * std::sin(lon),
        std::cos(lat),
    });
    const Vec3 vdir = normalize3({
        north.x * std::cos(az) + east.x * std::sin(az),
        north.y * std::cos(az) + east.y * std::sin(az),
        north.z * std::cos(az) + east.z * std::sin(az),
    });
    const Vec3 h = cross3(rhat, vdir);
    const Vec3 node{-h.y, h.x, 0.0};
    const double n = std::sqrt(dot3(node, node));
    if (n <= 1e-10) return wrap360_deg(rad2deg(lon));
    return wrap360_deg(rad2deg(std::atan2(node.y, node.x)));
}

std::vector<LaunchWindowSample> build_launch_window_samples(
    const MissionRequest& request,
    const OrbitTarget& orbit_target,
    double& selected_offset_s,
    double& selected_earth_rotation_angle_deg,
    double& selected_raan_deg,
    double& target_raan_deg,
    double& selected_plane_error_deg) {
    const bool locked_raan = std::isfinite(request.target_raan_deg);
    const double current_rotation = wrap360_deg(request.earth_rotation_angle_deg);
    target_raan_deg = locked_raan
        ? wrap360_deg(request.target_raan_deg)
        : launch_raan_deg_for_rotation(request, orbit_target, current_rotation);

    const double half_width_s = clampd(request.launch_window_half_width_min, 1.0, 720.0) * 60.0;
    const double step_s = (half_width_s <= 10.0 * 60.0) ? 30.0 : 60.0;
    const double window_tol_deg = 0.25;

    std::vector<LaunchWindowSample> samples;
    samples.reserve(static_cast<size_t>(2.0 * half_width_s / step_s) + 3);

    LaunchWindowSample best;
    for (double offset_s = -half_width_s; offset_s <= half_width_s + 1e-9; offset_s += step_s) {
        LaunchWindowSample s;
        s.offset_s = offset_s;
        s.earth_rotation_angle_deg = wrap360_deg(current_rotation + rad2deg(kOmega * offset_s));
        s.launch_raan_deg = launch_raan_deg_for_rotation(request, orbit_target, s.earth_rotation_angle_deg);
        s.plane_error_deg = angle_diff_deg(s.launch_raan_deg, target_raan_deg);
        s.score = std::abs(s.plane_error_deg);
        s.in_window = s.score <= window_tol_deg;
        if (!std::isfinite(best.score) ||
            s.score < best.score - 1e-9 ||
            (std::abs(s.score - best.score) <= 1e-9 && std::abs(s.offset_s) < std::abs(best.offset_s))) {
            best = s;
        }
        samples.push_back(s);
    }

    if (!locked_raan) {
        for (const LaunchWindowSample& s : samples) {
            if (std::abs(s.offset_s) <= 1e-9) {
                best = s;
                break;
            }
        }
    }

    selected_offset_s = best.offset_s;
    selected_earth_rotation_angle_deg = best.earth_rotation_angle_deg;
    selected_raan_deg = best.launch_raan_deg;
    selected_plane_error_deg = best.plane_error_deg;
    return samples;
}

double initial_downrange_velocity(const MissionRequest& request, const OrbitTarget& orbit_target) {
    const double lat = deg2rad(request.lat_deg);
    const double az = deg2rad(orbit_target.launch_az_deg);
    return kOmega * kRe * std::cos(lat) * std::sin(az);
}

Vec3 atmosphere_velocity_eci(const Vec3& r) {
    return cross3({0.0, 0.0, kOmega}, r);
}

LocalFrame3D local_frame(const FlatState& s, const OrbitTarget& orbit_target) {
    LocalFrame3D out;
    out.rhat = normalize3(s.r);
    const Vec3 n_hat = normalize3(orbit_target.plane_normal_eci);
    out.that = normalize3(cross3(n_hat, out.rhat));
    if (dot3(out.that, orbit_target.launch_tangent_eci) < 0.0) out.that *= -1.0;
    if (dot3(out.that, out.that) < 1e-10) {
        out.that = normalize3(reject3(orbit_target.launch_tangent_eci, out.rhat));
    }
    out.chat = normalize3(cross3(out.rhat, out.that));
    return out;
}

double altitude_m(const FlatState& s) {
    return norm3(s.r) - kRe;
}

double downrange_m(const FlatState& s, const OrbitTarget& orbit_target) {
    const Vec3 rhat = normalize3(s.r);
    const double along = dot3(rhat, orbit_target.launch_tangent_eci);
    const double radial = dot3(rhat, orbit_target.launch_rhat_eci);
    return kRe * std::atan2(along, radial);
}

PolarState polar_from_state3d(const FlatState& s, const OrbitTarget& orbit_target) {
    const LocalFrame3D frame = local_frame(s, orbit_target);
    const double r = norm3(s.r);
    const double vr = dot3(s.v, frame.rhat);
    const double vt = dot3(s.v, frame.that);
    return {r, downrange_m(s, orbit_target) / kRe, vr, vt, s.m};
}

Vec3 thrust_dir_from_rpy(const RpyCommand& cmd, const LocalFrame3D& frame) {
    const double cp = std::cos(cmd.pitch_rad);
    const double sp = std::sin(cmd.pitch_rad);
    const double cy = std::cos(cmd.yaw_rad);
    const double sy = std::sin(cmd.yaw_rad);
    return normalize3(frame.that * (cp * cy) + frame.chat * (cp * sy) + frame.rhat * sp);
}

RpyCommand interpolate_rpy_table(const std::vector<RpyTablePoint>& table, double t_s, const RpyCommand& fallback) {
    if (table.empty()) return fallback;
    if (t_s <= table.front().t_s) return table.front().rpy;
    if (t_s >= table.back().t_s) return table.back().rpy;
    for (size_t i = 1; i < table.size(); ++i) {
        if (table[i].t_s < t_s) continue;
        const RpyTablePoint& a = table[i - 1];
        const RpyTablePoint& b = table[i];
        const double u = clampd((t_s - a.t_s) / std::max(1e-9, b.t_s - a.t_s), 0.0, 1.0);
        return {
            lerpd(a.rpy.roll_rad, b.rpy.roll_rad, u),
            lerpd(a.rpy.pitch_rad, b.rpy.pitch_rad, u),
            lerpd(a.rpy.yaw_rad, b.rpy.yaw_rad, u),
        };
    }
    return fallback;
}

Quat interpolate_quat_table(const std::vector<QuatTablePoint>& table, double t_s) {
    if (table.empty()) return {};
    if (t_s <= table.front().t_s) return table.front().q;
    if (t_s >= table.back().t_s) return table.back().q;
    for (size_t i = 1; i < table.size(); ++i) {
        if (table[i].t_s < t_s) continue;
        const QuatTablePoint& a = table[i - 1];
        const QuatTablePoint& b = table[i];
        const double u = clampd((t_s - a.t_s) / std::max(1e-9, b.t_s - a.t_s), 0.0, 1.0);
        return slerp_quat(a.q, b.q, u);
    }
    return table.back().q;
}

RpyCommand evaluate_steering_model(const SteeringModel3D& model, double t_s, const RpyCommand& guidance) {
    RpyCommand model_cmd = guidance;
    switch (model.type) {
        case SteeringModelType::GuidanceRpy:
            return guidance;
        case SteeringModelType::RpyPolynomial:
            model_cmd = {
                model.roll_poly.value(t_s),
                model.pitch_poly.value(t_s),
                model.yaw_poly.value(t_s),
            };
            break;
        case SteeringModelType::RpyTable:
            model_cmd = interpolate_rpy_table(model.rpy_table, t_s, guidance);
            break;
        case SteeringModelType::QuaternionTable:
            return guidance;
    }
    const double u = clampd(model.model_blend, 0.0, 1.0);
    return {
        lerpd(guidance.roll_rad, model_cmd.roll_rad, u),
        lerpd(guidance.pitch_rad, model_cmd.pitch_rad, u),
        lerpd(guidance.yaw_rad, model_cmd.yaw_rad, u),
    };
}

double interpolate_throttle_table(const std::vector<ThrottleTablePoint>& table, double t_s, double fallback) {
    if (table.empty()) return fallback;
    if (t_s <= table.front().t_s) return table.front().throttle;
    if (t_s >= table.back().t_s) return table.back().throttle;
    for (size_t i = 1; i < table.size(); ++i) {
        if (table[i].t_s < t_s) continue;
        const ThrottleTablePoint& a = table[i - 1];
        const ThrottleTablePoint& b = table[i];
        const double u = clampd((t_s - a.t_s) / std::max(1e-9, b.t_s - a.t_s), 0.0, 1.0);
        return lerpd(a.throttle, b.throttle, u);
    }
    return fallback;
}

double evaluate_throttle_model(const ThrottleModel& model, double t_s, double guidance) {
    double model_cmd = guidance;
    switch (model.type) {
        case ThrottleModelType::Guidance:
            return guidance;
        case ThrottleModelType::Constant:
            model_cmd = model.constant;
            break;
        case ThrottleModelType::Polynomial:
            model_cmd = model.polynomial.value(t_s);
            break;
        case ThrottleModelType::Table:
            model_cmd = interpolate_throttle_table(model.table, t_s, guidance);
            break;
    }
    return clampd(lerpd(guidance, model_cmd, clampd(model.model_blend, 0.0, 1.0)), 0.0, 1.0);
}

void push_sample(std::vector<LvdSample>& samples, const FlatState& s, const OrbitTarget& orbit_target, double t, double q_kpa, double throttle) {
    const LocalFrame3D frame = local_frame(s, orbit_target);
    const double r = norm3(s.r);
    const double alt = std::max(0.0, r - kRe);
    const double vr = dot3(s.v, frame.rhat);
    const double horizontal_speed = norm3(s.v - frame.rhat * vr);
    const double speed = norm3(s.v);
    samples.push_back({
        t,
        downrange_m(s, orbit_target) / 1000.0,
        alt / 1000.0,
        speed,
        rad2deg(std::atan2(vr, std::max(1.0, horizontal_speed))),
        q_kpa,
        throttle,
        s.m,
        {s.r, s.v, s.m, true},
    });
}

void append_series_point(Series& s, double t_s, double value) {
    if (std::isfinite(t_s) && std::isfinite(value)) s.pts.push_back({t_s, value});
}

double interp3(double x, double x0, double y0, double x1, double y1, double x2, double y2) {
    if (x <= x1) {
        const double u = clampd((x - x0) / std::max(1e-9, x1 - x0), 0.0, 1.0);
        return lerpd(y0, y1, u);
    }
    const double u = clampd((x - x1) / std::max(1e-9, x2 - x1), 0.0, 1.0);
    return lerpd(y1, y2, u);
}

std::vector<Series> build_time_series(const std::vector<LvdSample>& samples) {
    Series alt;
    alt.name = L"LVD Altitude (km)";
    alt.color = RGB(41, 128, 185);
    Series speed;
    speed.name = L"LVD Speed (m/s)";
    speed.color = RGB(192, 57, 43);
    Series gamma;
    gamma.name = L"LVD Flight Path (deg)";
    gamma.color = RGB(142, 68, 173);
    Series q;
    q.name = L"LVD Dynamic Pressure (kPa)";
    q.color = RGB(211, 84, 0);
    Series throttle;
    throttle.name = L"LVD Throttle (%)";
    throttle.color = RGB(39, 174, 96);

    for (const LvdSample& s : samples) {
        append_series_point(alt, s.t_s, s.alt_km);
        append_series_point(speed, s.t_s, s.speed_mps);
        append_series_point(gamma, s.t_s, s.gamma_deg);
        append_series_point(q, s.t_s, s.q_kpa);
        append_series_point(throttle, s.t_s, 100.0 * s.throttle);
    }

    return {std::move(alt), std::move(speed), std::move(gamma), std::move(q), std::move(throttle)};
}

std::vector<LvdStateSample> build_state_samples(const std::vector<LvdSample>& samples) {
    std::vector<LvdStateSample> out;
    out.reserve(samples.size());
    for (const LvdSample& s : samples) {
        const double gamma = deg2rad(s.gamma_deg);
        LvdStateSample sample;
        sample.t_s = s.t_s;
        sample.state.r = kRe + std::max(0.0, s.alt_km) * 1000.0;
        sample.state.theta = (s.downrange_km * 1000.0) / kRe;
        sample.state.vr = s.speed_mps * std::sin(gamma);
        sample.state.vt = s.speed_mps * std::cos(gamma);
        sample.state.m = s.mass_kg;
        sample.state3d = s.state3d;
        sample.q_kpa = s.q_kpa;
        sample.throttle = s.throttle;
        out.push_back(sample);
    }
    return out;
}

LvdSample sample_near_time(const std::vector<LvdSample>& samples, double t_s) {
    if (samples.empty()) return {};
    const LvdSample* best = &samples.front();
    double best_dt = std::abs(best->t_s - t_s);
    for (const LvdSample& s : samples) {
        const double dt = std::abs(s.t_s - t_s);
        if (dt < best_dt) {
            best = &s;
            best_dt = dt;
        }
    }
    return *best;
}

LvdEvent make_event(const std::wstring& name, const std::vector<LvdSample>& samples, double t_s) {
    const LvdSample s = sample_near_time(samples, t_s);
    return {name, t_s, s.alt_km, s.downrange_km, s.speed_mps, s.gamma_deg, s.q_kpa, s.throttle, s.mass_kg};
}

LvdResult make_score_lvd_result(const CandidateSim& sim) {
    LvdResult out;
    out.stage1 = sim.stage1;
    out.state_samples = build_state_samples(sim.samples);
    out.target_sep_alt_km = sim.design.sep_alt_km;
    out.target_sep_speed_mps = sim.design.sep_speed_mps;
    out.target_sep_gamma_deg = sim.design.sep_gamma_deg;
    return out;
}

double score_candidate_with_mission(const LvdOptions& options, const CandidateSim& sim) {
    if (!options.mission_score) return sim.score;
    if (!std::isfinite(sim.score) || sim.samples.empty()) return std::numeric_limits<double>::infinity();
    const LvdResult probe = make_score_lvd_result(sim);
    const double score = options.mission_score(probe);
    return std::isfinite(score) ? score : std::numeric_limits<double>::infinity();
}

LvdDesign clamp_design(const LvdDesign& design, double max_burn_s, bool force_stage1_burnout) {
    LvdDesign out = design;
    out.burn_s = force_stage1_burnout ? max_burn_s : clampd(out.burn_s, 90.0, max_burn_s);
    out.sep_alt_km = clampd(out.sep_alt_km, 34.0, 112.0);
    out.sep_speed_mps = clampd(out.sep_speed_mps, 1600.0, 3900.0);
    out.sep_gamma_deg = clampd(out.sep_gamma_deg, -8.0, 18.0);
    out.pitch_bias_early_deg = clampd(out.pitch_bias_early_deg, -8.0, 8.0);
    out.pitch_bias_mid_deg = clampd(out.pitch_bias_mid_deg, -10.0, 10.0);
    out.pitch_bias_late_deg = clampd(out.pitch_bias_late_deg, -10.0, 10.0);
    out.pitch_bias_terminal_deg = clampd(out.pitch_bias_terminal_deg, -10.0, 10.0);
    out.pitch_min_bias_deg = clampd(out.pitch_min_bias_deg, -5.0, 5.0);
    out.q_soft_bias_kpa = clampd(out.q_soft_bias_kpa, -8.0, 8.0);
    out.throttle_floor_mid_bias = clampd(out.throttle_floor_mid_bias, -0.12, 0.08);
    out.throttle_floor_late_bias = clampd(out.throttle_floor_late_bias, -0.12, 0.08);
    out.q_bucket_gain = clampd(out.q_bucket_gain, 0.010, 0.035);
    out.pitch_rate_scale = clampd(out.pitch_rate_scale, 0.75, 1.35);
    out.terminal_gain_scale = clampd(out.terminal_gain_scale, 0.70, 1.35);
    return out;
}

bool design_close(const LvdDesign& a, const LvdDesign& b) {
    return std::abs(a.burn_s - b.burn_s) < 0.05 &&
           std::abs(a.sep_alt_km - b.sep_alt_km) < 0.05 &&
           std::abs(a.sep_speed_mps - b.sep_speed_mps) < 1.0 &&
           std::abs(a.sep_gamma_deg - b.sep_gamma_deg) < 0.02 &&
           std::abs(a.pitch_bias_early_deg - b.pitch_bias_early_deg) < 0.02 &&
           std::abs(a.pitch_bias_mid_deg - b.pitch_bias_mid_deg) < 0.02 &&
           std::abs(a.pitch_bias_late_deg - b.pitch_bias_late_deg) < 0.02 &&
           std::abs(a.pitch_bias_terminal_deg - b.pitch_bias_terminal_deg) < 0.02 &&
           std::abs(a.pitch_min_bias_deg - b.pitch_min_bias_deg) < 0.02 &&
           std::abs(a.q_soft_bias_kpa - b.q_soft_bias_kpa) < 0.02 &&
           std::abs(a.throttle_floor_mid_bias - b.throttle_floor_mid_bias) < 0.002 &&
           std::abs(a.throttle_floor_late_bias - b.throttle_floor_late_bias) < 0.002 &&
           std::abs(a.q_bucket_gain - b.q_bucket_gain) < 0.0005 &&
           std::abs(a.pitch_rate_scale - b.pitch_rate_scale) < 0.002 &&
           std::abs(a.terminal_gain_scale - b.terminal_gain_scale) < 0.002;
}

void push_internal_seed(std::vector<CandidateSim>& seeds, CandidateSim cand, size_t limit) {
    if (!std::isfinite(cand.score)) return;
    for (const CandidateSim& seed : seeds) {
        if (design_close(seed.design, cand.design)) return;
    }
    seeds.push_back(std::move(cand));
    std::sort(seeds.begin(), seeds.end(), [](const CandidateSim& a, const CandidateSim& b) {
        if (a.score != b.score) return a.score < b.score;
        return a.stage1.rem_prop > b.stage1.rem_prop;
    });
    if (seeds.size() > limit) seeds.resize(limit);
}

bool mission_scored_better(const MissionScoredSim& cand, const MissionScoredSim& best) {
    if (!std::isfinite(cand.mission_score)) return false;
    if (!std::isfinite(best.mission_score)) return true;
    if (cand.mission_score < best.mission_score - 1e-9) return true;
    if (cand.mission_score > best.mission_score + 1e-9) return false;
    if (cand.sim.score < best.sim.score - 1e-9) return true;
    if (cand.sim.score > best.sim.score + 1e-9) return false;
    return cand.sim.stage1.rem_prop > best.sim.stage1.rem_prop + 1e-6;
}

std::vector<LvdEvent> build_events(const CandidateSim& sim) {
    std::vector<LvdEvent> events;
    events.reserve(5);
    events.push_back(make_event(L"Liftoff", sim.samples, 0.0));
    events.push_back(make_event(L"LVD Closed-Loop Pitch", sim.samples, sim.stage1.guide_start_s));
    events.push_back(make_event(L"Max-Q", sim.samples, sim.stage1.t_max_q));
    events.push_back(make_event(L"MECO", sim.samples, sim.stage1.meco_s));
    events.push_back(make_event(L"1st Stage SEP", sim.samples, sim.stage1.sep_s));
    return events;
}

CandidateSim simulate_lvd_design(
    const MissionRequest& request,
    const OrbitTarget& orbit_target,
    const LvdOptions& options,
    const LvdDesign& design) {
    CandidateSim out;
    out.design = design;

    FlatState s;
    s.r = orbit_target.launch_rhat_eci * kRe;
    s.v = atmosphere_velocity_eci(s.r);
    s.m = request.s1_dry_kg + request.s1_prop_kg + request.s2_dry_kg + request.s2_prop_kg + request.payload_kg;
    const Vec3 plane_normal = normalize3(orbit_target.plane_normal_eci);

    const double thrust = request.s1_thrust_kN * 1000.0;
    const double mdot_nom = thrust / std::max(1e-6, request.s1_isp_s * kG0);
    const double s1_reserve_ratio = clampd(request.s1_reserve, 0.0, 0.40);
    const double s1_max_consumable_kg = options.allow_full_burn
        ? request.s1_prop_kg
        : std::max(0.0, request.s1_prop_kg * (1.0 - s1_reserve_ratio));
    const double burn_max_s = s1_max_consumable_kg / std::max(1e-6, mdot_nom);
    const double sep_delay_s = clampd(request.s1_sep_delay_s, 1.5, 6.0);
    const double burn_s = options.force_stage1_burnout
        ? burn_max_s
        : clampd(design.burn_s, 80.0, burn_max_s);
    const double cda = 9.0;
    const double dt_guidance = 0.20;
    const double q_soft_kpa = clampd(request.s1_target_maxq_kpa + design.q_soft_bias_kpa, 15.0, request.q_limit_kpa);
    const double guide_start_s = 8.0;
    const double payload_shape = clampd((request.payload_kg - 6500.0) / 6500.0, 0.0, 1.0);

    const double sep_target_z = design.sep_alt_km * 1000.0;
    const double sep_target_vz = design.sep_speed_mps * std::sin(deg2rad(design.sep_gamma_deg));
    const double sep_target_vx = design.sep_speed_mps * std::cos(deg2rad(design.sep_gamma_deg));
    const double coast_g = grav(sep_target_z);
    const double meco_target_z = std::max(0.0, sep_target_z - sep_target_vz * sep_delay_s + 0.5 * coast_g * sep_delay_s * sep_delay_s);
    const double meco_target_vz = sep_target_vz + coast_g * sep_delay_s;

    out.stage1.guide_start_s = guide_start_s;
    out.stage1.min_throttle = 1.0;
    out.stage1.traj.reserve(360);
    out.stage1.traj3d.reserve(360);
    out.samples.reserve(360);
    out.stage1.traj.push_back({0.0, 0.0, 0.0});
    out.stage1.traj3d.push_back({0.0, s.r, s.v});
    push_sample(out.samples, s, orbit_target, 0.0, 0.0, 1.0);

    double t = 0.0;
    double used = 0.0;
    double pitch_cmd = deg2rad(89.2);
    double yaw_cmd = 0.0;
    double last_q_kpa = 0.0;
    double last_throttle = 1.0;

    while (t < burn_s - 1e-9 && used < s1_max_consumable_kg - 1e-9) {
        const double dt = std::min(dt_guidance, burn_s - t);
        const LocalFrame3D frame = local_frame(s, orbit_target);
        const double alt_m = std::max(0.0, altitude_m(s));
        const double speed = norm3(s.v);
        const Vec3 air_v = s.v - atmosphere_velocity_eci(s.r);
        const double air_speed = norm3(air_v);
        const double dens = rho(alt_m);
        const double q_kpa = 0.5 * dens * air_speed * air_speed / 1000.0;
        last_q_kpa = q_kpa;
        if (q_kpa > out.stage1.max_q) {
            out.stage1.max_q = q_kpa;
            out.stage1.t_max_q = t;
        }

        const double r_norm = std::max(1.0, norm3(s.r));
        const Vec3 gravity = s.r * (-kMu / (r_norm * r_norm * r_norm));
        const double drag = 0.5 * dens * air_speed * air_speed * cda;
        Vec3 drag_acc{0.0, 0.0, 0.0};
        if (air_speed > 1e-6) {
            const double invm = 1.0 / std::max(1.0, s.m);
            drag_acc = air_v * ((drag * invm) / air_speed);
        }

        const double vr = dot3(s.v, frame.rhat);
        const double vt = dot3(s.v, frame.that);
        const double cross_pos = dot3(s.r, plane_normal);
        const double cross_vel = dot3(s.v, plane_normal);
        const double tgo = clampd(burn_s - t, 4.0, 120.0);
        const double ax_need = clampd(design.terminal_gain_scale * (sep_target_vx - vt) / tgo, -30.0, 35.0);
        const double az_need = clampd(
            design.terminal_gain_scale * ((meco_target_vz - vr) / tgo +
                2.0 * (meco_target_z - alt_m - vr * tgo) / std::max(1.0, tgo * tgo)),
            -30.0,
            38.0);
        const double ac_need = clampd(
            design.terminal_gain_scale * (-0.85 * cross_vel / tgo - 1.30 * cross_pos / std::max(1.0, tgo * tgo)),
            -8.0,
            8.0);
        double thrust_ax_cmd = ax_need + dot3(drag_acc, frame.that);
        double thrust_az_cmd = az_need - dot3(gravity, frame.rhat) + dot3(drag_acc, frame.rhat);
        double thrust_ac_cmd = ac_need + dot3(drag_acc, plane_normal);
        thrust_ax_cmd = std::max(1e-4, thrust_ax_cmd);
        thrust_az_cmd = std::max(0.0, thrust_az_cmd);
        double pitch_raw = std::atan2(thrust_az_cmd, thrust_ax_cmd);
        double yaw_raw = std::atan2(thrust_ac_cmd, std::max(1e-4, std::hypot(thrust_ax_cmd, thrust_az_cmd)));
        const double vertical_hold = 1.0 - smoothstep(6.0, 22.0, t);
        pitch_raw = lerpd(pitch_raw, deg2rad(89.2), vertical_hold);
        yaw_raw = lerpd(yaw_raw, 0.0, vertical_hold);
        const double burn_u = clampd(t / std::max(1.0, burn_s), 0.0, 1.0);
        const double pitch_bias_deg = interp3(
            burn_u,
            0.18,
            design.pitch_bias_early_deg,
            0.55,
            design.pitch_bias_mid_deg,
            0.88,
            design.pitch_bias_late_deg);
        const double terminal_pitch_bias_deg = lerpd(
            pitch_bias_deg,
            design.pitch_bias_terminal_deg,
            smoothstep(0.78, 0.98, burn_u));
        pitch_raw += deg2rad(terminal_pitch_bias_deg);
        const double pitch_floor_light_deg =
            (t < 10.0) ? 88.0 :
            (alt_m < 3500.0) ? 78.0 :
            (alt_m < 9000.0) ? 40.0 :
            (alt_m < 18000.0) ? 12.0 : 2.0;
        const double pitch_floor_heavy_deg =
            (t < 10.0) ? 88.0 :
            (alt_m < 4000.0) ? 80.0 :
            (alt_m < 12000.0) ? 58.0 :
            (alt_m < 26000.0) ? 28.0 :
            (alt_m < 45000.0) ? 11.0 : 3.0;
        const double pitch_floor_deg = lerpd(pitch_floor_light_deg, pitch_floor_heavy_deg, payload_shape) + terminal_pitch_bias_deg;
        pitch_raw = std::max(pitch_raw, deg2rad(pitch_floor_deg));
        const double pitch_min_deg =
            ((payload_shape > 0.45 && t > 90.0 && alt_m > 42000.0) ? -6.0 : 2.0) + design.pitch_min_bias_deg;
        pitch_raw = clampd(pitch_raw, deg2rad(pitch_min_deg), deg2rad(89.4));
        yaw_raw = clampd(yaw_raw, deg2rad(-15.0), deg2rad(15.0));

        const double rate_limit_deg_s =
            ((t < 35.0) ? 1.45 :
            (t < 70.0) ? 2.25 : 3.30) * design.pitch_rate_scale;
        const double dmax = deg2rad(rate_limit_deg_s) * dt;
        pitch_cmd += clampd(pitch_raw - pitch_cmd, -dmax, dmax);
        pitch_cmd = clampd(pitch_cmd, deg2rad(pitch_min_deg), deg2rad(89.4));
        const double yaw_dmax = deg2rad(std::max(0.75, 0.70 * rate_limit_deg_s)) * dt;
        yaw_cmd += clampd(yaw_raw - yaw_cmd, -yaw_dmax, yaw_dmax);
        yaw_cmd = clampd(yaw_cmd, deg2rad(-15.0), deg2rad(15.0));

        const double acc_cmd = std::sqrt(thrust_ax_cmd * thrust_ax_cmd + thrust_az_cmd * thrust_az_cmd + thrust_ac_cmd * thrust_ac_cmd);
        const double thrust_acc_max = thrust / std::max(1.0, s.m);
        double throttle_cmd = clampd(acc_cmd / std::max(1e-6, thrust_acc_max), 0.35, 1.0);
        const double throttle_floor =
            clampd(
                ((alt_m < 2000.0) ? 1.0 :
                (alt_m < 18000.0) ? 0.82 :
                (alt_m < 35000.0) ? 0.74 : 0.45) +
                    lerpd(
                        design.throttle_floor_mid_bias,
                        design.throttle_floor_late_bias,
                        smoothstep(0.50, 0.90, burn_u)),
                0.35,
                1.0);
        throttle_cmd = std::max(throttle_cmd, throttle_floor);
        if (q_kpa > q_soft_kpa) {
            const double q_bucket = clampd(1.0 - design.q_bucket_gain * (q_kpa - q_soft_kpa), 0.54, 1.0);
            throttle_cmd = std::min(throttle_cmd, q_bucket);
        }
        if (t < 12.0) throttle_cmd = 1.0;
        if (t > 45.0 && q_kpa < 0.85 * q_soft_kpa) throttle_cmd = std::max(throttle_cmd, 0.96);
        if (q_kpa > request.q_limit_kpa) {
            throttle_cmd = std::min(throttle_cmd, clampd(0.90 - 0.030 * (q_kpa - request.q_limit_kpa), 0.45, 0.90));
        }
        throttle_cmd = evaluate_throttle_model(options.throttle_model, t, throttle_cmd);
        last_throttle = throttle_cmd;

        if (throttle_cmd < out.stage1.min_throttle - 1e-9) {
            out.stage1.min_throttle = throttle_cmd;
            out.stage1.t_min_throttle = t;
        }

        const double dm = std::min(mdot_nom * throttle_cmd * dt, s1_max_consumable_kg - used);
        const double mdot = dm / std::max(1e-9, dt);
        const double thrust_now = mdot * request.s1_isp_s * kG0;
        const double invm = 1.0 / std::max(1.0, s.m);
        RpyCommand rpy_cmd = evaluate_steering_model(options.steering_model, t, {0.0, pitch_cmd, yaw_cmd});
        Vec3 thrust_dir = thrust_dir_from_rpy(rpy_cmd, frame);
        if (options.steering_model.type == SteeringModelType::QuaternionTable &&
            !options.steering_model.quat_table.empty()) {
            thrust_dir = normalize3(rotate_quat(interpolate_quat_table(options.steering_model.quat_table, t), {1.0, 0.0, 0.0}));
        }
        const Vec3 accel = thrust_dir * (thrust_now * invm) + gravity - drag_acc;

        s.v += accel * dt;
        s.r += s.v * dt;
        s.m -= dm;
        used += dm;
        t += dt;
        if (altitude_m(s) < 0.0) {
            const Vec3 rhat = normalize3(s.r);
            s.r = rhat * kRe;
            const double vr_surface = dot3(s.v, rhat);
            if (vr_surface < 0.0) s.v -= rhat * vr_surface;
        }

        if (t - out.stage1.traj.back().t >= 1.0 || t >= burn_s - 1e-6) {
            out.stage1.traj.push_back({t, downrange_m(s, orbit_target) / 1000.0, std::max(0.0, altitude_m(s) / 1000.0)});
            out.stage1.traj3d.push_back({t, s.r, s.v});
            push_sample(out.samples, s, orbit_target, t, q_kpa, throttle_cmd);
        }
    }

    out.stage1.meco = polar_from_state3d(s, orbit_target);
    out.stage1.meco3d = {s.r, s.v, s.m, true};
    out.stage1.meco_s = t;
    out.stage1.burn_s = t;
    out.stage1.used_prop = used;
    out.stage1.rem_prop = std::max(0.0, request.s1_prop_kg - used);

    double coast_elapsed = 0.0;
    while (coast_elapsed < sep_delay_s - 1e-9) {
        const double dt = std::min(0.20, sep_delay_s - coast_elapsed);
        const double alt_m = std::max(0.0, altitude_m(s));
        const Vec3 air_v = s.v - atmosphere_velocity_eci(s.r);
        const double air_speed = norm3(air_v);
        const double dens = rho(alt_m);
        const double r_norm = std::max(1.0, norm3(s.r));
        const Vec3 gravity = s.r * (-kMu / (r_norm * r_norm * r_norm));
        const double drag = 0.5 * dens * air_speed * air_speed * cda;
        Vec3 drag_acc{0.0, 0.0, 0.0};
        if (air_speed > 1e-6) {
            const double invm = 1.0 / std::max(1.0, s.m);
            drag_acc = air_v * ((drag * invm) / air_speed);
        }
        s.v += (gravity - drag_acc) * dt;
        s.r += s.v * dt;
        t += dt;
        coast_elapsed += dt;
        if (altitude_m(s) < 0.0) {
            const Vec3 rhat = normalize3(s.r);
            s.r = rhat * kRe;
            const double vr_surface = dot3(s.v, rhat);
            if (vr_surface < 0.0) s.v -= rhat * vr_surface;
        }
        if (t - out.stage1.traj.back().t >= 1.0 || coast_elapsed >= sep_delay_s - 1e-6) {
            out.stage1.traj.push_back({t, downrange_m(s, orbit_target) / 1000.0, std::max(0.0, altitude_m(s) / 1000.0)});
            out.stage1.traj3d.push_back({t, s.r, s.v});
            push_sample(out.samples, s, orbit_target, t, last_q_kpa, last_throttle);
        }
    }

    out.stage1.sep = polar_from_state3d(s, orbit_target);
    out.stage1.sep3d = {s.r, s.v, s.m, true};
    out.stage1.sep_s = t;

    const double sep_alt_km = (out.stage1.sep.r - kRe) / 1000.0;
    const double sep_speed_mps = std::hypot(out.stage1.sep.vr, out.stage1.sep.vt);
    const double sep_gamma_deg = rad2deg(std::atan2(out.stage1.sep.vr, std::max(1.0, out.stage1.sep.vt)));
    out.stage1.target_alt_err_km = sep_alt_km - design.sep_alt_km;
    out.stage1.target_speed_err_mps = sep_speed_mps - design.sep_speed_mps;
    out.stage1.target_gamma_err_deg = sep_gamma_deg - design.sep_gamma_deg;
    out.stage1.tgo_final_s = 0.0;
    out.stage1.vgo_final_mps = std::hypot(sep_target_vx - out.stage1.sep.vt, sep_target_vz - out.stage1.sep.vr);
    out.stage1.converged =
        std::isfinite(sep_alt_km) &&
        std::isfinite(sep_speed_mps) &&
        std::isfinite(sep_gamma_deg) &&
        std::isfinite(out.stage1.max_q);
    out.stage1.envelope_ok =
        out.stage1.max_q <= request.q_limit_kpa + 1e-6 &&
        sep_alt_km >= 35.0 &&
        sep_alt_km <= 115.0 &&
        sep_speed_mps >= 1500.0 &&
        sep_speed_mps <= 3800.0 &&
        sep_gamma_deg >= -6.0 &&
        sep_gamma_deg <= 22.0;

    const double reserve_goal_kg = options.allow_full_burn ? 0.0 : request.s1_prop_kg * s1_reserve_ratio;
    out.score =
        std::abs(out.stage1.target_alt_err_km) * 5.0 +
        std::abs(out.stage1.target_speed_err_mps) * 0.030 +
        std::abs(out.stage1.target_gamma_err_deg) * 9.0 +
        std::max(0.0, out.stage1.max_q - request.q_limit_kpa) * 260.0 +
        std::max(0.0, reserve_goal_kg - out.stage1.rem_prop) * 0.045;
    if (!out.stage1.envelope_ok) {
        out.score += 5000.0;
        out.score += std::max(0.0, 35.0 - sep_alt_km) * 80.0;
        out.score += std::max(0.0, sep_alt_km - 115.0) * 55.0;
        out.score += std::max(0.0, 1500.0 - sep_speed_mps) * 0.8;
        out.score += std::max(0.0, sep_speed_mps - 3800.0) * 0.6;
        out.score += std::max(0.0, -6.0 - sep_gamma_deg) * 120.0;
        out.score += std::max(0.0, sep_gamma_deg - 22.0) * 100.0;
    }
    if (!out.stage1.converged) out.score = std::numeric_limits<double>::infinity();
    return out;
}

CandidateSim choose_best_lvd_design(
    const MissionRequest& request,
    const OrbitTarget& orbit_target,
    const LvdOptions& options,
    double base_alt_km,
    double base_speed_mps,
    double base_gamma_deg) {
    const double thrust = request.s1_thrust_kN * 1000.0;
    const double mdot = thrust / std::max(1e-6, request.s1_isp_s * kG0);
    const double max_consumable = options.allow_full_burn
        ? request.s1_prop_kg
        : request.s1_prop_kg * (1.0 - clampd(request.s1_reserve, 0.0, 0.40));
    const double max_burn_s = max_consumable / std::max(1e-6, mdot);
    const double base_burn_s = options.force_stage1_burnout
        ? max_burn_s
        : clampd(request.s1_target_meco_s, 95.0, std::max(95.0, max_burn_s));

    const std::array<double, 7> burn_offsets = {-20.0, -12.0, -5.0, 0.0, 6.0, 12.0, 20.0};
    const std::array<double, 3> alt_offsets = {-12.0, 0.0, 12.0};
    const std::array<double, 5> speed_offsets = {-360.0, -160.0, 0.0, 180.0, 380.0};
    const std::array<double, 3> gamma_offsets = {-2.5, 0.0, 2.5};

    CandidateSim best;
    std::vector<CandidateSim> internal_seeds;
    const size_t seed_limit = options.mission_score
        ? static_cast<size_t>(std::max(12, std::min(32, options.max_mission_design_evals)))
        : 1u;
    auto consider = [&](const LvdDesign& design) {
        if (cancellation_requested(options)) return;
        CandidateSim cand = simulate_lvd_design(
            request,
            orbit_target,
            options,
            clamp_design(design, max_burn_s, options.force_stage1_burnout));
        if (cand.score < best.score - 1e-9 ||
            (std::abs(cand.score - best.score) <= 1e-9 && cand.stage1.rem_prop > best.stage1.rem_prop + 1e-6)) {
            best = cand;
        }
        push_internal_seed(internal_seeds, std::move(cand), seed_limit);
    };

    if (options.force_stage1_burnout) {
        consider({max_burn_s, base_alt_km, base_speed_mps, base_gamma_deg});
    } else {
        for (double db : burn_offsets) {
            for (double da : alt_offsets) {
                for (double dv : speed_offsets) {
                    for (double dg : gamma_offsets) {
                        LvdDesign design;
                        design.burn_s = base_burn_s + db;
                        design.sep_alt_km = base_alt_km + da;
                        design.sep_speed_mps = base_speed_mps + dv;
                        design.sep_gamma_deg = base_gamma_deg + dg;
                        consider(design);
                    }
                }
            }
        }
    }

    if (!options.mission_score || options.max_mission_design_evals <= 0 || internal_seeds.empty()) {
        return best;
    }

    MissionScoredSim best_mission;
    int evals = 0;
    const int max_evals = std::max(1, options.max_mission_design_evals);
    auto eval_design = [&](const LvdDesign& raw_design) {
        MissionScoredSim out;
        if (cancellation_requested(options) || evals >= max_evals) return out;
        const LvdDesign design = clamp_design(raw_design, max_burn_s, options.force_stage1_burnout);
        out.sim = simulate_lvd_design(request, orbit_target, options, design);
        out.mission_score = score_candidate_with_mission(options, out.sim);
        ++evals;
        if (std::isfinite(out.mission_score)) out.sim.score = out.mission_score;
        return out;
    };
    auto eval_existing = [&](const CandidateSim& sim) {
        MissionScoredSim out;
        if (cancellation_requested(options) || evals >= max_evals) return out;
        out.sim = sim;
        out.mission_score = score_candidate_with_mission(options, out.sim);
        ++evals;
        if (std::isfinite(out.mission_score)) out.sim.score = out.mission_score;
        return out;
    };

    const int seed_evals = std::min<int>(static_cast<int>(internal_seeds.size()), std::max(8, max_evals / 3));
    for (int i = 0; i < seed_evals; ++i) {
        MissionScoredSim scored = eval_existing(internal_seeds[static_cast<size_t>(i)]);
        if (mission_scored_better(scored, best_mission)) best_mission = std::move(scored);
    }
    if (!std::isfinite(best_mission.mission_score)) return best;

    std::array<double, 15> step{8.0, 7.0, 180.0, 1.6, 3.5, 4.5, 4.5, 4.0, 2.0, 3.0, 0.05, 0.05, 0.006, 0.12, 0.14};
    const std::array<double, 15> min_step{1.0, 1.0, 35.0, 0.25, 0.45, 0.55, 0.55, 0.50, 0.35, 0.45, 0.008, 0.008, 0.001, 0.025, 0.030};
    while (!cancellation_requested(options) && evals < max_evals) {
        bool improved = false;
        for (size_t dim = 0; dim < step.size() && evals < max_evals; ++dim) {
            if (step[dim] < min_step[dim]) continue;
            for (double sign : {-1.0, 1.0}) {
                if (evals >= max_evals) break;
                LvdDesign trial = best_mission.sim.design;
                if (dim == 0) trial.burn_s += sign * step[dim];
                if (dim == 1) trial.sep_alt_km += sign * step[dim];
                if (dim == 2) trial.sep_speed_mps += sign * step[dim];
                if (dim == 3) trial.sep_gamma_deg += sign * step[dim];
                if (dim == 4) trial.pitch_bias_early_deg += sign * step[dim];
                if (dim == 5) trial.pitch_bias_mid_deg += sign * step[dim];
                if (dim == 6) trial.pitch_bias_late_deg += sign * step[dim];
                if (dim == 7) trial.pitch_bias_terminal_deg += sign * step[dim];
                if (dim == 8) trial.pitch_min_bias_deg += sign * step[dim];
                if (dim == 9) trial.q_soft_bias_kpa += sign * step[dim];
                if (dim == 10) trial.throttle_floor_mid_bias += sign * step[dim];
                if (dim == 11) trial.throttle_floor_late_bias += sign * step[dim];
                if (dim == 12) trial.q_bucket_gain += sign * step[dim];
                if (dim == 13) trial.pitch_rate_scale += sign * step[dim];
                if (dim == 14) trial.terminal_gain_scale += sign * step[dim];
                MissionScoredSim scored = eval_design(trial);
                if (mission_scored_better(scored, best_mission)) {
                    best_mission = std::move(scored);
                    improved = true;
                }
            }
        }
        if (!improved) {
            bool any_active = false;
            for (size_t i = 0; i < step.size(); ++i) {
                step[i] *= 0.55;
                any_active = any_active || step[i] >= min_step[i];
            }
            if (!any_active) break;
        }
    }
    return (std::isfinite(best_mission.mission_score) &&
            best_mission.mission_score < kAcceptedMissionScoreCutoff)
        ? std::move(best_mission.sim)
        : best;
}

}  // namespace

LvdResult solve_stage1_lvd(
    const MissionRequest& request,
    const OrbitTarget& orbit_target,
    const LvdOptions& options) {
    LvdResult out;

    out.launch_window_samples = build_launch_window_samples(
        request,
        orbit_target,
        out.launch_offset_s,
        out.earth_rotation_angle_deg,
        out.launch_raan_deg,
        out.target_raan_deg,
        out.plane_error_deg);

    const double orbit_span_km = std::max(0.0, orbit_target.ra_km - orbit_target.rp_km);
    const double cutoff_delta_km = orbit_target.cutoff_alt_km - 200.0;
    const double payload_shape = clampd((request.payload_kg - 6500.0) / 6500.0, 0.0, 1.0);
    const double base_alt_km = clampd(
        50.0 + 0.030 * cutoff_delta_km + 0.002 * orbit_span_km + 16.0 * payload_shape,
        38.0,
        88.0);
    const double base_speed_mps = clampd(
        3250.0 + 0.45 * cutoff_delta_km + 0.16 * orbit_span_km + 160.0 * payload_shape,
        2600.0,
        3700.0);
    const double base_gamma_deg = clampd(
        0.8 + 0.0025 * orbit_span_km + 4.5 * payload_shape,
        -1.0,
        9.5);

    CandidateSim best = choose_best_lvd_design(
        request,
        orbit_target,
        options,
        base_alt_km,
        base_speed_mps,
        base_gamma_deg);

    out.stage1 = std::move(best.stage1);
    out.target_sep_alt_km = best.design.sep_alt_km;
    out.target_sep_speed_mps = best.design.sep_speed_mps;
    out.target_sep_gamma_deg = best.design.sep_gamma_deg;
    out.time_series = build_time_series(best.samples);
    out.state_samples = build_state_samples(best.samples);
    out.events = build_events(best);
    return out;
}

}  // namespace falcon9
