#pragma once

#define NOMINMAX
#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace falcon9 {

constexpr double kG0 = 9.80665;
constexpr double kRe = 6378137.0;
constexpr double kMu = 3.986004418e14;
constexpr double kMuKm = 3.986004418e5;
constexpr double kOmega = 7.2921159e-5;

inline double clampd(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

inline double lerpd(double a, double b, double u) {
    return a + (b - a) * u;
}

inline double smoothstep(double edge0, double edge1, double x) {
    if (edge1 <= edge0) return (x >= edge1) ? 1.0 : 0.0;
    const double u = clampd((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return u * u * (3.0 - 2.0 * u);
}

inline double deg2rad(double d) {
    return d * 3.14159265358979323846 / 180.0;
}

inline double rad2deg(double r) {
    return r * 180.0 / 3.14159265358979323846;
}

inline double grav(double alt_m) {
    const double r = kRe + std::max(0.0, alt_m);
    return kMu / std::max(1.0, r * r);
}

inline double rho(double alt_m) {
    const double h = std::max(0.0, alt_m);
    if (h > 120000.0) return 0.0;
    return 1.225 * std::exp(-h / 8500.0);
}

inline double wrap_lon_deg(double lon_deg) {
    double out = lon_deg;
    while (out > 180.0) out -= 360.0;
    while (out < -180.0) out += 360.0;
    return out;
}

inline double wrap360_deg(double deg) {
    double out = std::fmod(deg, 360.0);
    if (out < 0.0) out += 360.0;
    return out;
}

inline double angle_diff_deg(double a, double b) {
    double d = wrap360_deg(a) - wrap360_deg(b);
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

inline void destination_from_course(
    double lat1_deg,
    double lon1_deg,
    double az_deg,
    double dist_km,
    double& lat2_deg,
    double& lon2_deg) {
    const double r_earth_km = kRe / 1000.0;
    const double sigma = dist_km / std::max(1e-6, r_earth_km);
    const double lat1 = deg2rad(lat1_deg);
    const double lon1 = deg2rad(lon1_deg);
    const double az = deg2rad(az_deg);

    const double sin_lat1 = std::sin(lat1);
    const double cos_lat1 = std::cos(lat1);
    const double sin_sigma = std::sin(sigma);
    const double cos_sigma = std::cos(sigma);

    const double sin_lat2 = sin_lat1 * cos_sigma + cos_lat1 * sin_sigma * std::cos(az);
    const double lat2 = std::asin(clampd(sin_lat2, -1.0, 1.0));

    const double y = std::sin(az) * sin_sigma * cos_lat1;
    const double x = cos_sigma - sin_lat1 * std::sin(lat2);
    const double lon2 = lon1 + std::atan2(y, x);

    lat2_deg = rad2deg(lat2);
    lon2_deg = wrap_lon_deg(rad2deg(lon2));
}

inline double direct_launch_min_incl_deg(double lat_deg) {
    return clampd(std::abs(lat_deg), 0.0, 90.0);
}

inline double direct_launch_target_incl_deg(double lat_deg, double incl_deg) {
    const double min_incl = direct_launch_min_incl_deg(lat_deg);
    const double req_incl = clampd(std::abs(incl_deg), 0.0, 180.0);
    return clampd(req_incl, min_incl, 180.0 - min_incl);
}

inline double direct_launch_sin_az(double lat_deg, double incl_deg) {
    const double lat = deg2rad(lat_deg);
    const double target_incl = deg2rad(direct_launch_target_incl_deg(lat_deg, incl_deg));
    const double c_lat = std::cos(lat);
    if (std::abs(c_lat) <= 1e-6) return 0.0;
    return clampd(std::cos(target_incl) / c_lat, -1.0, 1.0);
}

inline double direct_launch_azimuth_deg(double lat_deg, double incl_deg) {
    return rad2deg(std::asin(direct_launch_sin_az(lat_deg, incl_deg)));
}

inline double direct_launch_effective_incl_deg(double lat_deg, double launch_az_deg) {
    const double lat = deg2rad(lat_deg);
    const double az = deg2rad(launch_az_deg);
    return rad2deg(std::acos(clampd(std::cos(lat) * std::sin(az), -1.0, 1.0)));
}

inline double prop_for_dv(double m0, double dv, double isp) {
    if (dv <= 0.0 || isp <= 1e-6 || m0 <= 1.0) return 0.0;
    const double mf = m0 / std::exp(dv / (isp * kG0));
    return std::max(0.0, m0 - mf);
}

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 operator-(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 operator*(const Vec3& v, double s) {
    return {v.x * s, v.y * s, v.z * s};
}

inline Vec3 operator*(double s, const Vec3& v) {
    return v * s;
}

inline Vec3 operator/(const Vec3& v, double s) {
    return {v.x / s, v.y / s, v.z / s};
}

inline Vec3& operator+=(Vec3& a, const Vec3& b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

inline Vec3& operator-=(Vec3& a, const Vec3& b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

inline Vec3& operator*=(Vec3& v, double s) {
    v.x *= s;
    v.y *= s;
    v.z *= s;
    return v;
}

inline double dot3(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline double norm3(const Vec3& v) {
    return std::sqrt(dot3(v, v));
}

inline Vec3 cross3(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

inline Vec3 normalize3(const Vec3& v) {
    const double n = norm3(v);
    if (n <= 1e-12) return {0.0, 0.0, 0.0};
    return {v.x / n, v.y / n, v.z / n};
}

inline Vec3 reject3(const Vec3& v, const Vec3& n_hat) {
    return v - n_hat * dot3(v, n_hat);
}

inline Vec3 ecef_from_geo(double lat_deg, double lon_deg, double alt_km) {
    const double lat = deg2rad(lat_deg);
    const double lon = deg2rad(lon_deg);
    const double r = 1.0 + alt_km / (kRe / 1000.0);
    const double cos_lat = std::cos(lat);
    return {
        r * cos_lat * std::cos(lon),
        r * cos_lat * std::sin(lon),
        r * std::sin(lat),
    };
}

struct Quat {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline Quat normalize_quat(const Quat& q) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n <= 1e-12) return {};
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

inline Quat slerp_quat(Quat a, Quat b, double u) {
    a = normalize_quat(a);
    b = normalize_quat(b);
    double d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    if (d < 0.0) {
        d = -d;
        b = {-b.w, -b.x, -b.y, -b.z};
    }
    u = clampd(u, 0.0, 1.0);
    if (d > 0.9995) {
        return normalize_quat({
            lerpd(a.w, b.w, u),
            lerpd(a.x, b.x, u),
            lerpd(a.y, b.y, u),
            lerpd(a.z, b.z, u),
        });
    }
    const double theta0 = std::acos(clampd(d, -1.0, 1.0));
    const double theta = theta0 * u;
    const double s0 = std::cos(theta) - d * std::sin(theta) / std::max(1e-12, std::sin(theta0));
    const double s1 = std::sin(theta) / std::max(1e-12, std::sin(theta0));
    return normalize_quat({
        s0 * a.w + s1 * b.w,
        s0 * a.x + s1 * b.x,
        s0 * a.y + s1 * b.y,
        s0 * a.z + s1 * b.z,
    });
}

inline Vec3 rotate_quat(const Quat& q_in, const Vec3& v) {
    const Quat q = normalize_quat(q_in);
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 uv = cross3(u, v);
    const Vec3 uuv = cross3(u, uv);
    return v + (uv * (2.0 * q.w)) + (uuv * 2.0);
}

struct StateVector3D {
    Vec3 r_m{kRe, 0.0, 0.0};
    Vec3 v_mps{0.0, 0.0, 0.0};
    double m_kg = 0.0;
    bool valid = false;
};

struct SimPt3D {
    double t = 0.0;
    Vec3 r_m{kRe, 0.0, 0.0};
    Vec3 v_mps{0.0, 0.0, 0.0};
};

struct RpyCommand {
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
};

struct CubicPoly {
    double c0 = 0.0;
    double c1 = 0.0;
    double c2 = 0.0;
    double c3 = 0.0;

    double value(double t_s) const {
        return ((c3 * t_s + c2) * t_s + c1) * t_s + c0;
    }
};

struct RpyTablePoint {
    double t_s = 0.0;
    RpyCommand rpy;
};

struct QuatTablePoint {
    double t_s = 0.0;
    Quat q;
};

enum class SteeringModelType {
    GuidanceRpy,
    RpyPolynomial,
    RpyTable,
    QuaternionTable,
};

struct SteeringModel3D {
    SteeringModelType type = SteeringModelType::GuidanceRpy;
    CubicPoly roll_poly;
    CubicPoly pitch_poly;
    CubicPoly yaw_poly;
    std::vector<RpyTablePoint> rpy_table;
    std::vector<QuatTablePoint> quat_table;
    double model_blend = 1.0;
};

struct ThrottleTablePoint {
    double t_s = 0.0;
    double throttle = 1.0;
};

enum class ThrottleModelType {
    Guidance,
    Constant,
    Polynomial,
    Table,
};

struct ThrottleModel {
    ThrottleModelType type = ThrottleModelType::Guidance;
    double constant = 1.0;
    CubicPoly polynomial;
    std::vector<ThrottleTablePoint> table;
    double model_blend = 1.0;
};

struct MissionRequest {
    double payload_kg = 15000.0;
    double perigee_km = 200.0;
    double apogee_km = 200.0;
    double cutoff_alt_km = std::numeric_limits<double>::quiet_NaN();
    double incl_deg = 28.5;
    double lat_deg = 28.5;
    double launch_lon_deg = -80.6;
    double earth_rotation_angle_deg = 0.0;
    double target_raan_deg = std::numeric_limits<double>::quiet_NaN();
    double launch_epoch_utc_jd = std::numeric_limits<double>::quiet_NaN();
    double launch_window_half_width_min = 45.0;
    double ship_downrange_km = 620.0;
    double losses_mps = 1500.0;
    double q_limit_kpa = 45.0;
    double s1_target_maxq_kpa = 35.0;
    double s1_target_meco_s = 145.0;
    double s1_sep_delay_s = 3.0;
    double s1_dry_kg = 25600.0;
    double s1_prop_kg = 395700.0;
    double s1_isp_s = 282.0;
    double s1_thrust_kN = 7607.0;
    double s1_reserve = 0.0;
    double s2_dry_kg = 4000.0;
    double s2_prop_kg = 92670.0;
    double s2_isp_s = 348.0;
    double s2_thrust_kN = 981.0;
    double s2_ignition_delay_s = 7.0;
    double s2_target_seco_s = 529.0;
};

struct PlotPt {
    double x_km = 0.0;
    double y_km = 0.0;
};

struct Series {
    std::wstring name;
    COLORREF color = RGB(0, 0, 0);
    std::vector<PlotPt> pts;
};

struct GlobePt {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double alt_km = 0.0;
};

struct GlobeSeries {
    std::wstring name;
    COLORREF color = RGB(0, 0, 0);
    std::vector<GlobePt> pts;
};

struct SimPt {
    double t = 0.0;
    double x_km = 0.0;
    double z_km = 0.0;
};

struct PolarState {
    double r = kRe;
    double theta = 0.0;
    double vr = 0.0;
    double vt = 0.0;
    double m = 0.0;
};

struct OrbitMetrics {
    double rp_km = 0.0;
    double ra_km = 0.0;
    double a_km = 0.0;
    double e = 0.0;
    double speed_mps = 0.0;
    double flight_path_deg = 0.0;
};

struct OrbitTarget {
    double rp_km = 0.0;
    double ra_km = 0.0;
    double cutoff_alt_km = 0.0;
    double launch_az_deg = 90.0;
    double r_target_m = kRe;
    double vr_target_mps = 0.0;
    double vt_target_mps = 0.0;
    double speed_target_mps = 0.0;
    double fpa_target_deg = 0.0;
    Vec3 launch_rhat_eci{1.0, 0.0, 0.0};
    Vec3 launch_east_eci{0.0, 1.0, 0.0};
    Vec3 launch_north_eci{0.0, 0.0, 1.0};
    Vec3 launch_tangent_eci{0.0, 1.0, 0.0};
    Vec3 plane_normal_eci{0.0, 0.0, 1.0};
    bool has_3d_plane = false;
};

struct Stage1Result {
    PolarState meco;
    PolarState sep;
    StateVector3D meco3d;
    StateVector3D sep3d;
    double burn_s = 0.0;
    double meco_s = 0.0;
    double sep_s = 0.0;
    double guide_start_s = 0.0;
    double used_prop = 0.0;
    double rem_prop = 0.0;
    double max_q = 0.0;
    double t_max_q = 0.0;
    double min_throttle = 1.0;
    double t_min_throttle = 0.0;
    bool converged = false;
    bool envelope_ok = false;
    double target_alt_err_km = 0.0;
    double target_speed_err_mps = 0.0;
    double target_gamma_err_deg = 0.0;
    double tgo_final_s = 0.0;
    double vgo_final_mps = 0.0;
    std::vector<SimPt> traj;
    std::vector<SimPt3D> traj3d;
};

struct Stage2Result {
    PolarState ignition;
    PolarState seco;
    StateVector3D ignition3d;
    StateVector3D seco3d;
    OrbitMetrics orbit;
    double ignition_s = 0.0;
    double burn_s = 0.0;
    double cutoff_s = 0.0;
    double used_prop = 0.0;
    double rem_prop = 0.0;
    double target_r_err_km = 0.0;
    double target_rp_err_km = 0.0;
    double target_ra_err_km = 0.0;
    double target_fpa_err_deg = 0.0;
    double peak_alt_km = 0.0;
    double orbit_penalty = std::numeric_limits<double>::infinity();
    bool converged = false;
    bool orbit_ok = false;
    double tgo_final_s = 0.0;
    double vgo_final_mps = 0.0;
    std::vector<SimPt> traj;
    std::vector<SimPt3D> traj3d;
};

struct RecoveryResult {
    bool feasible = false;
    bool converged = false;
    double landing_ignition_time_s = std::numeric_limits<double>::quiet_NaN();
    double landing_prop_kg = 0.0;
    double touchdown_time_s = 0.0;
    double touchdown_downrange_km = 0.0;
    double touchdown_lat_deg = 0.0;
    double touchdown_lon_deg = 0.0;
    double margin_kg = -std::numeric_limits<double>::infinity();
    double touchdown_speed_mps = 0.0;
    std::vector<SimPt> coast_traj;
    std::vector<SimPt> landing_traj;
};

struct LvdEvent {
    std::wstring name;
    double t_s = 0.0;
    double alt_km = 0.0;
    double downrange_km = 0.0;
    double speed_mps = 0.0;
    double flight_path_deg = 0.0;
    double q_kpa = 0.0;
    double throttle = 0.0;
    double mass_kg = 0.0;
};

struct LaunchWindowSample {
    double offset_s = 0.0;
    double earth_rotation_angle_deg = 0.0;
    double launch_raan_deg = 0.0;
    double plane_error_deg = 0.0;
    double score = std::numeric_limits<double>::infinity();
    bool in_window = false;
};

struct LvdStateSample {
    double t_s = 0.0;
    PolarState state;
    StateVector3D state3d;
    double q_kpa = 0.0;
    double throttle = 0.0;
};

struct SeparationCandidate {
    double sep_time_s = 0.0;
    double sep_alt_target_km = 0.0;
    double sep_speed_target_mps = 0.0;
    double sep_gamma_target_deg = 0.0;
    Stage1Result stage1;
    Stage2Result stage2;
    RecoveryResult recovery;
    bool feasible = false;
    double orbit_miss_score = std::numeric_limits<double>::infinity();
    double recovery_surplus_kg = std::numeric_limits<double>::infinity();
    double score = std::numeric_limits<double>::infinity();
};

struct MissionResult {
    bool ok = false;
    bool payload_search_ok = false;
    std::wstring status;
    std::vector<std::wstring> lines;
    std::vector<Series> profile_series;
    std::vector<Series> separation_time_series;
    std::vector<Series> lvd_time_series;
    std::vector<SeparationCandidate> separation_candidates;
    std::vector<LvdEvent> lvd_events;
    std::vector<LaunchWindowSample> launch_window_samples;
    std::vector<GlobeSeries> globe_series;
    double launch_lat_deg = 0.0;
    double launch_lon_deg = -80.6;
    double launch_epoch_utc_jd = std::numeric_limits<double>::quiet_NaN();
    double lvd_launch_offset_s = 0.0;
    double lvd_earth_rotation_angle_deg = 0.0;
    double lvd_launch_raan_deg = 0.0;
    double lvd_target_raan_deg = std::numeric_limits<double>::quiet_NaN();
    double lvd_plane_error_deg = 0.0;
    double ship_lat_deg = 0.0;
    double ship_lon_deg = -80.0;
    double view_lat_deg = 20.0;
    double view_lon_deg = -60.0;
    OrbitTarget orbit_target;
    SeparationCandidate best_candidate;
    Stage1Result stage1;
    Stage2Result stage2;
    RecoveryResult recovery;
};

enum class SeparationSearchMode {
    RefinedDefault,
    Coarse1s,
};

struct SolveControl {
    const std::atomic<bool>* cancel_requested = nullptr;
    unsigned worker_count = 0;
    SeparationSearchMode separation_search_mode = SeparationSearchMode::RefinedDefault;
    bool ignore_recovery = false;
    bool force_stage1_burnout = false;
};

OrbitMetrics orbit_metrics_from_state(const PolarState& s);
OrbitMetrics orbit_metrics_from_state3d(const StateVector3D& s);
OrbitTarget build_orbit_target(const MissionRequest& request);
MissionRequest sanitize_request(const MissionRequest& request);
MissionResult solve_mission(const MissionRequest& request, SolveControl control = {});

inline bool finite_plot_pt(const PlotPt& p) {
    return std::isfinite(p.x_km) && std::isfinite(p.y_km);
}

inline bool finite_globe_pt(const GlobePt& p) {
    return std::isfinite(p.lat_deg) && std::isfinite(p.lon_deg) && std::isfinite(p.alt_km);
}

inline bool mission_payload_search_ok(const MissionResult& result) {
    return result.payload_search_ok;
}

}  // namespace falcon9
