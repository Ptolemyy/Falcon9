#define NOMINMAX
#include <windows.h>
#include <commctrl.h>
#include <windowsx.h>

#include <algorithm>
#include <cmath>
#include <cwchar>
#include <cwctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#include <shellapi.h>

namespace {

constexpr double kG0 = 9.80665;
constexpr double kRe = 6378137.0;
constexpr double kMu = 3.986004418e14;
constexpr double kOmega = 7.2921159e-5;

constexpr int kBtnPlan = 1001;
constexpr int kList = 1002;
constexpr int kBtnMaxPayload = 1003;
constexpr int kFieldBase = 1100;

struct Input {
    double payload_kg = 15000.0;
    double perigee_km = 200.0;
    double apogee_km = 200.0;
    double incl_deg = 28.5;
    double lat_deg = 28.5;
    double launch_lon_deg = -80.6;
    double ship_downrange_km = 620.0;
    double losses_mps = 1500.0;
    double q_limit_kpa = 40.0;
    double s1_dry_kg = 25600.0;
    double s1_prop_kg = 395700.0;
    double s1_isp_s = 282.0;
    double s1_thrust_kN = 7607.0;
    double s1_reserve = 0.08;
    double s2_dry_kg = 4000.0;
    double s2_prop_kg = 92670.0;
    double s2_isp_s = 348.0;
    double s2_thrust_kN = 981.0;
};

struct Field {
    int id = 0;
    const wchar_t* label = L"";
    const wchar_t* unit = L"";
    double dval = 0.0;
    double Input::*member = nullptr;
    HWND h_label = nullptr;
    HWND h_edit = nullptr;
    HWND h_unit = nullptr;
};

struct State {
    double x = 0.0;
    double z = 0.0;
    double vx = 0.0;
    double vz = 0.0;
    double m = 0.0;
};

struct SimPt {
    double t = 0.0;
    double x_km = 0.0;
    double z_km = 0.0;
};

struct Stage1Out {
    State sep;
    double burn_s = 0.0;
    double used_prop = 0.0;
    double rem_prop = 0.0;
    double max_q = 0.0;
    double t_max_q = 0.0;
    std::vector<SimPt> traj;
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

struct Plan {
    bool ok = false;
    std::wstring status;
    std::vector<std::wstring> lines;
    std::vector<Series> profile_series;
    std::vector<GlobeSeries> globe_series;
    double launch_lat_deg = 0.0;
    double launch_lon_deg = -80.6;
    double ship_lat_deg = 0.0;
    double ship_lon_deg = -80.0;
    double view_lat_deg = 20.0;
    double view_lon_deg = -60.0;
};

struct App {
    std::vector<Field> fields;
    HWND btn = nullptr;
    HWND btn_max_payload = nullptr;
    HWND list = nullptr;
    HFONT font = nullptr;
    RECT plot{0, 0, 0, 0};
    RECT globe_panel{0, 0, 0, 0};
    bool view_initialized = false;
    double view_lat_deg = 20.0;
    double view_lon_deg = -60.0;
    bool dragging_globe = false;
    POINT last_mouse{0, 0};
    Input base_input;
    bool has_vehicle_config = false;
    std::wstring vehicle_config_path;
    std::wstring vehicle_config_error;
    Plan plan;
};

struct StartupVehicleConfig {
    bool has_config = false;
    std::wstring config_path;
    std::wstring error;
    Input input;
};
StartupVehicleConfig g_startup_vehicle;

double clampd(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

double deg2rad(double d) {
    return d * 3.14159265358979323846 / 180.0;
}

double grav(double alt_m) {
    const double r = kRe + std::max(0.0, alt_m);
    return kMu / (r * r);
}

double rho(double alt_m) {
    const double h = std::max(0.0, alt_m);
    if (h > 120000.0) return 0.0;
    return 1.225 * std::exp(-h / 8500.0);
}

std::wstring trim(const std::wstring& s) {
    size_t b = 0;
    size_t e = s.size();
    while (b < e && std::iswspace(static_cast<wint_t>(s[b]))) ++b;
    while (e > b && std::iswspace(static_cast<wint_t>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

bool parse_num(const std::wstring& s, double& out) {
    const std::wstring t = trim(s);
    if (t.empty()) return false;
    wchar_t* end = nullptr;
    const double v = std::wcstod(t.c_str(), &end);
    if (!end || end == t.c_str()) return false;
    while (*end && std::iswspace(static_cast<wint_t>(*end))) ++end;
    if (*end != L'\0' || !std::isfinite(v)) return false;
    out = v;
    return true;
}

std::wstring fnum(double v, int p) {
    std::wostringstream oss;
    oss << std::fixed << std::setprecision(p) << v;
    return oss.str();
}

std::wstring ftime(double sec) {
    const int t = std::max(0, static_cast<int>(std::lround(sec)));
    const int m = t / 60;
    const int s = t % 60;
    wchar_t buf[32];
    std::swprintf(buf, 32, L"T+%02d:%02d", m, s);
    return buf;
}

double prop_for_dv(double m0, double dv, double isp) {
    if (m0 <= 0.0 || dv <= 0.0 || isp <= 1e-6) return 0.0;
    const double mf = m0 / std::exp(dv / (isp * kG0));
    return clampd(m0 - mf, 0.0, m0);
}

std::vector<PlotPt> bezier(const PlotPt& p0, const PlotPt& p1, const PlotPt& p2, const PlotPt& p3, int n) {
    std::vector<PlotPt> out;
    n = std::max(8, n);
    out.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(n - 1);
        const double a = 1.0 - u;
        const double b0 = a * a * a;
        const double b1 = 3.0 * a * a * u;
        const double b2 = 3.0 * a * u * u;
        const double b3 = u * u * u;
        out.push_back({b0 * p0.x_km + b1 * p1.x_km + b2 * p2.x_km + b3 * p3.x_km,
                       b0 * p0.y_km + b1 * p1.y_km + b2 * p2.y_km + b3 * p3.y_km});
    }
    return out;
}

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

double dot3(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 normalize3(const Vec3& v) {
    const double n = std::sqrt(dot3(v, v));
    if (n <= 1e-12) return {0.0, 0.0, 0.0};
    return {v.x / n, v.y / n, v.z / n};
}

Vec3 cross3(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

double azimuth_from_north_east(double north, double east) {
    double az = std::atan2(east, north) * 180.0 / 3.14159265358979323846;
    while (az > 180.0) az -= 360.0;
    while (az < -180.0) az += 360.0;
    return az;
}

double ang_diff_deg(double a, double b) {
    double d = a - b;
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return std::abs(d);
}

std::string trim_ascii_copy(std::string s) {
    auto is_ws = [](char c) {
        return c == ' ' || c == '\t' || c == '\r' || c == '\n';
    };
    size_t b = 0;
    size_t e = s.size();
    while (b < e && is_ws(s[b])) ++b;
    while (e > b && is_ws(s[e - 1])) --e;
    return s.substr(b, e - b);
}

std::string lower_ascii_copy(std::string s) {
    for (char& c : s) {
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    return s;
}

bool apply_vehicle_value(Input& out, const std::string& key_in, double v) {
    const std::string key = lower_ascii_copy(trim_ascii_copy(key_in));
    if (key.empty()) return false;

    if (key == "payload_kg") out.payload_kg = v;
    else if (key == "perigee_km" || key == "orbit_perigee_km") out.perigee_km = v;
    else if (key == "apogee_km" || key == "orbit_apogee_km") out.apogee_km = v;
    else if (key == "incl_deg" || key == "inclination_deg" || key == "orbit_inclination_deg") out.incl_deg = v;
    else if (key == "lat_deg" || key == "launch_lat_deg" || key == "launch_latitude_deg") out.lat_deg = v;
    else if (key == "launch_lon_deg" || key == "lon_deg" || key == "launch_longitude_deg") out.launch_lon_deg = v;
    else if (key == "ship_downrange_km" || key == "droneship_downrange_km") out.ship_downrange_km = v;
    else if (key == "losses_mps" || key == "ascent_losses_mps") out.losses_mps = v;
    else if (key == "q_limit_kpa" || key == "max_q_limit_kpa") out.q_limit_kpa = v;

    else if (key == "s1_dry_kg" || key == "stage1_dry_kg") out.s1_dry_kg = v;
    else if (key == "s1_prop_kg" || key == "stage1_prop_kg") out.s1_prop_kg = v;
    else if (key == "s1_isp_s" || key == "stage1_isp_s") out.s1_isp_s = v;
    else if (key == "s1_thrust_kn" || key == "stage1_thrust_kn") out.s1_thrust_kN = v;
    else if (key == "s1_reserve" || key == "stage1_reserve" || key == "stage1_reserve_ratio") out.s1_reserve = v;

    else if (key == "s2_dry_kg" || key == "stage2_dry_kg") out.s2_dry_kg = v;
    else if (key == "s2_prop_kg" || key == "stage2_prop_kg") out.s2_prop_kg = v;
    else if (key == "s2_isp_s" || key == "stage2_isp_s") out.s2_isp_s = v;
    else if (key == "s2_thrust_kn" || key == "stage2_thrust_kn") out.s2_thrust_kN = v;
    else return false;

    return true;
}

bool load_vehicle_config(const std::wstring& path, Input& out, std::wstring& err) {
    std::ifstream in(std::filesystem::path(path), std::ios::in);
    if (!in) {
        err = L"Cannot open vehicle config file.";
        return false;
    }

    Input cfg = Input{};
    int applied = 0;
    std::string line;
    while (std::getline(in, line)) {
        if (const size_t p = line.find('#'); p != std::string::npos) line = line.substr(0, p);
        if (const size_t p = line.find("//"); p != std::string::npos) line = line.substr(0, p);
        line = trim_ascii_copy(line);
        if (line.empty()) continue;

        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = trim_ascii_copy(line.substr(0, eq));
        std::string val = trim_ascii_copy(line.substr(eq + 1));
        if (key.empty() || val.empty()) continue;

        try {
            double v = std::stod(val);
            if (apply_vehicle_value(cfg, key, v)) applied++;
        } catch (...) {
            continue;
        }
    }

    if (applied <= 0) {
        err = L"Vehicle config has no valid numeric key=value entries.";
        return false;
    }
    out = cfg;
    return true;
}

double wrap_lon_deg(double lon_deg) {
    double out = lon_deg;
    while (out > 180.0) out -= 360.0;
    while (out < -180.0) out += 360.0;
    return out;
}

void destination_from_course(
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

    lat2_deg = lat2 * 180.0 / 3.14159265358979323846;
    lon2_deg = wrap_lon_deg(lon2 * 180.0 / 3.14159265358979323846);
}

Vec3 ecef_from_geo(double lat_deg, double lon_deg, double alt_km) {
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

Stage1Out sim_stage1(const Input& in, double burn_s, double prop_limit, double m0);
Stage1Out search_stage1(const Input& in);
Plan build_plan(const Input& in);
std::vector<Field> make_fields(const Input& seed);
void split_plot_rects(const RECT& outer, RECT& left, RECT& right);
bool load_vehicle_config(const std::wstring& path, Input& out, std::wstring& err);

Stage1Out sim_stage1(const Input& in, double burn_s, double prop_limit, double m0) {
    Stage1Out out;
    out.traj.reserve(240);
    State s;
    s.m = m0;

    const double thrust = in.s1_thrust_kN * 1000.0;
    const double mdot_nom = thrust / std::max(1e-6, in.s1_isp_s * kG0);
    const double cda = 10.0;
    double pitch_cmd_rad = deg2rad(89.0);

    out.traj.push_back({0.0, 0.0, 0.0});
    double t = 0.0;
    double used = 0.0;
    while (t < burn_s - 1e-9 && used < prop_limit - 1e-9) {
        const double dt = std::min(0.2, burn_s - t);
        const double dm = std::min(mdot_nom * dt, prop_limit - used);
        const double mdot = dm / std::max(1e-9, dt);
        const double thr = mdot * in.s1_isp_s * kG0;

        const double v = std::hypot(s.vx, s.vz);
        const double r = rho(s.z);
        const double q = 0.5 * r * v * v / 1000.0;
        if (q > out.max_q) {
            out.max_q = q;
            out.t_max_q = t;
        }

        const double g = grav(s.z);
        const double d = 0.5 * r * v * v * cda;
        const double invm = 1.0 / std::max(1.0, s.m);
        double drag_ax = 0.0;
        double drag_az = 0.0;
        if (v > 1e-6) {
            drag_ax = (d * invm) * (s.vx / v);
            drag_az = (d * invm) * (s.vz / v);
        }

        // ---- UPFG-like guidance law (simplified 2D form) ----
        // Solve a terminal-state driven thrust vector from remaining time-to-go.
        // This is not a strict flight software replica, but keeps the UPFG spirit:
        // command acceleration from terminal velocity/position objectives and t_go.
        const double t_go = std::max(0.6, burn_s - t);
        const double m_burnout_est = std::max(1.0, s.m - mdot_nom * t_go);
        const double dv_rem_ideal = in.s1_isp_s * kG0 * std::log(std::max(1.0001, s.m / m_burnout_est));
        const double v_orbit_70km = std::sqrt(kMu / (kRe + 70000.0));

        // Stage-1 pseudo terminal goals: near-horizontal high-energy handoff.
        const double z_target = clampd(62000.0 + 55.0 * std::max(0.0, t_go - 20.0), 60000.0, 95000.0);
        const double vx_target = clampd(0.30 * v_orbit_70km + 0.38 * dv_rem_ideal, 1400.0, 3600.0);
        const double vz_target = clampd(0.22 * (z_target - s.z) / t_go, -120.0, 220.0);

        const double ax_v = (vx_target - s.vx) / t_go;
        const double az_v = (vz_target - s.vz) / t_go;
        const double az_r = 2.0 * (z_target - s.z - s.vz * t_go) / (t_go * t_go);

        double ax_net_cmd = clampd(ax_v, -18.0, 35.0);
        double az_net_cmd = clampd(0.60 * az_v + 0.40 * az_r, -30.0, 26.0);

        // Max-Q protection: when dynamic pressure is high, bias to a more vertical vector.
        if (q > 0.85 * in.q_limit_kpa && t < 0.75 * burn_s) {
            az_net_cmd += 3.0;
            ax_net_cmd = std::min(ax_net_cmd, 10.0);
        }

        double thrust_ax_cmd = ax_net_cmd + drag_ax;
        double thrust_az_cmd = az_net_cmd + drag_az + g;
        if (thrust_ax_cmd < 1e-4) thrust_ax_cmd = 1e-4;
        if (thrust_az_cmd < 1e-4) thrust_az_cmd = 1e-4;

        double pitch_raw = std::atan2(thrust_az_cmd, thrust_ax_cmd);
        pitch_raw = clampd(pitch_raw, deg2rad(6.0), deg2rad(89.5));
        if (t < 8.0) pitch_raw = deg2rad(89.0);

        const double rate_limit_rad_s = deg2rad((t < 35.0) ? 1.8 : 1.2);
        const double dmax = rate_limit_rad_s * dt;
        pitch_cmd_rad += clampd(pitch_raw - pitch_cmd_rad, -dmax, dmax);
        pitch_cmd_rad = clampd(pitch_cmd_rad, deg2rad(6.0), deg2rad(89.5));

        double ax = (thr * std::cos(pitch_cmd_rad)) * invm;
        double az = (thr * std::sin(pitch_cmd_rad)) * invm - g;
        if (v > 1e-6) {
            ax -= drag_ax;
            az -= drag_az;
        }

        s.vx += ax * dt;
        s.vz += az * dt;
        s.x += s.vx * dt;
        s.z += s.vz * dt;
        if (s.z < 0.0) {
            s.z = 0.0;
            if (s.vz < 0.0) s.vz = 0.0;
        }

        s.m -= dm;
        used += dm;
        t += dt;

        if (t - out.traj.back().t >= 1.0 || t >= burn_s - 1e-6) {
            out.traj.push_back({t, s.x / 1000.0, s.z / 1000.0});
        }
    }

    out.sep = s;
    out.burn_s = t;
    out.used_prop = used;
    out.rem_prop = std::max(0.0, in.s1_prop_kg - used);
    return out;
}

struct Stage1CandidateEval {
    double score = std::numeric_limits<double>::infinity();
    double rec_margin_kg = -std::numeric_limits<double>::infinity();
    double s2_margin_kg = -std::numeric_limits<double>::infinity();
    double q_excess_kpa = 0.0;
    double sep_alt_km = 0.0;
    double sep_speed_mps = 0.0;
};

Stage1CandidateEval eval_stage1_candidate(const Input& in, const Stage1Out& o) {
    Stage1CandidateEval out;
    out.sep_alt_km = o.sep.z / 1000.0;
    out.sep_speed_mps = std::hypot(o.sep.vx, o.sep.vz);
    out.q_excess_kpa = std::max(0.0, o.max_q - in.q_limit_kpa);

    const double rp_km = std::min(in.perigee_km, in.apogee_km);
    const double ra_km = std::max(in.perigee_km, in.apogee_km);

    const double s2full = in.s2_dry_kg + in.s2_prop_kg + in.payload_kg;
    double s1_sep_m = o.sep.m - s2full;
    s1_sep_m = std::max(s1_sep_m, in.s1_dry_kg);

    const double gsep = grav(o.sep.z);
    const double disc = o.sep.vz * o.sep.vz + 2.0 * gsep * std::max(0.0, o.sep.z);
    double tfall = 0.0;
    if (disc > 0.0 && gsep > 1e-6) {
        tfall = (o.sep.vz + std::sqrt(disc)) / gsep;
        if (tfall < 0.0) tfall = std::sqrt(2.0 * o.sep.z / gsep);
    }
    const double iip_km = (o.sep.x + o.sep.vx * tfall) / 1000.0;
    const double iip_err = iip_km - in.ship_downrange_km;

    const double dv_corr = std::min(1200.0, std::abs(iip_err) * 1.7);
    const double dv_entry = 170.0 + 0.03 * std::max(0.0, out.sep_speed_mps - 1800.0);
    const double dv_land = 240.0 + 0.015 * std::max(0.0, out.sep_speed_mps - 1700.0);
    const double dv_rec_req = dv_corr + dv_entry + dv_land;
    const double rec_prop_need = prop_for_dv(s1_sep_m, dv_rec_req, in.s1_isp_s);
    out.rec_margin_kg = o.rem_prop - rec_prop_need;

    const double rp = kRe + rp_km * 1000.0;
    const double ra = kRe + ra_km * 1000.0;
    const double a = 0.5 * (rp + ra);
    const double v_orbit = std::sqrt(kMu * (2.0 / rp - 1.0 / a));

    const double lat = deg2rad(in.lat_deg);
    const double inc = deg2rad(in.incl_deg);
    const double c_lat = std::cos(lat);
    const double vrot = kOmega * kRe * c_lat;
    double plane_penalty = 0.0;
    double assist = 0.0;
    if (std::abs(c_lat) > 1e-6) {
        double sin_az = std::cos(inc) / c_lat;
        if (sin_az < -1.0 || sin_az > 1.0) {
            const double min_i = std::abs(in.lat_deg);
            const double di = std::max(0.0, min_i - std::abs(in.incl_deg));
            plane_penalty = 2.0 * v_orbit * std::sin(0.5 * deg2rad(di));
            sin_az = clampd(sin_az, -1.0, 1.0);
        }
        assist = vrot * std::max(0.0, sin_az);
    }

    const double dv_total = v_orbit + in.losses_mps - assist + plane_penalty;
    const double dv_s2_req = std::max(0.0, dv_total - out.sep_speed_mps) + 120.0;
    const double s2_need = prop_for_dv(s2full, dv_s2_req, in.s2_isp_s);
    out.s2_margin_kg = in.s2_prop_kg - s2_need;

    const double rec_def = std::max(0.0, -out.rec_margin_kg);
    const double s2_def = std::max(0.0, -out.s2_margin_kg);

    double score = 0.0;
    score += out.q_excess_kpa * 25.0;
    score += rec_def * 0.03;
    score += s2_def * 0.03;

    if (out.sep_alt_km < 20.0) score += (20.0 - out.sep_alt_km) * 8.0;
    if (out.sep_alt_km > 130.0) score += (out.sep_alt_km - 130.0) * 0.2;

    if (rec_def < 1e-6 && s2_def < 1e-6 && out.q_excess_kpa < 0.2) {
        score -= 0.0015 * (out.rec_margin_kg + out.s2_margin_kg);
    }
    out.score = score;
    return out;
}

Stage1Out search_stage1(const Input& in) {
    const double s2full = in.s2_dry_kg + in.s2_prop_kg + in.payload_kg;
    const double m0 = in.s1_dry_kg + in.s1_prop_kg + s2full;
    // Reserve is no longer a hard input constraint. Let optimization choose ascent burn,
    // and recovery feasibility naturally determines remaining propellant.
    const double prop_lim = in.s1_prop_kg;
    const double thrust = in.s1_thrust_kN * 1000.0;
    const double mdot = thrust / std::max(1e-6, in.s1_isp_s * kG0);
    const double burn_max = prop_lim / std::max(1e-6, mdot);
    const double lo = std::max(70.0, burn_max * 0.60);
    const double hi = std::max(lo + 5.0, burn_max);

    Stage1Out best{};
    double best_score = std::numeric_limits<double>::infinity();
    double best_burn = lo;

    for (int i = 0; i <= 48; ++i) {
        const double u = static_cast<double>(i) / 48.0;
        const double b = lo + (hi - lo) * u;
        Stage1Out o = sim_stage1(in, b, prop_lim, m0);
        const Stage1CandidateEval e = eval_stage1_candidate(in, o);
        if (e.score < best_score) {
            best_score = e.score;
            best_burn = b;
            best = std::move(o);
        }
    }
    const double rlo = std::max(lo, best_burn - 6.0);
    const double rhi = std::min(hi, best_burn + 6.0);
    for (int i = 0; i <= 48; ++i) {
        const double u = static_cast<double>(i) / 48.0;
        const double b = rlo + (rhi - rlo) * u;
        Stage1Out o = sim_stage1(in, b, prop_lim, m0);
        const Stage1CandidateEval e = eval_stage1_candidate(in, o);
        if (e.score < best_score) {
            best_score = e.score;
            best = std::move(o);
        }
    }
    return best;
}

double stage1_fall_time_sec(const Stage1Out& s1) {
    const double gsep = grav(s1.sep.z);
    const double disc = s1.sep.vz * s1.sep.vz + 2.0 * gsep * std::max(0.0, s1.sep.z);
    double tfall = 0.0;
    if (disc > 0.0 && gsep > 1e-6) {
        tfall = (s1.sep.vz + std::sqrt(disc)) / gsep;
        if (tfall < 0.0) tfall = std::sqrt(2.0 * s1.sep.z / gsep);
    }
    return tfall;
}

double stage1_iip_km(const Stage1Out& s1) {
    const double tfall = stage1_fall_time_sec(s1);
    return (s1.sep.x + s1.sep.vx * tfall) / 1000.0;
}

Plan build_plan(const Input& in) {
    Plan p;
    if (in.perigee_km < 80.0 || in.apogee_km < 80.0) {
        p.status = L"Invalid orbit";
        p.lines.push_back(L"Perigee/Apogee should be >= 80 km.");
        return p;
    }

    const double rp_km = std::min(in.perigee_km, in.apogee_km);
    const double ra_km = std::max(in.perigee_km, in.apogee_km);

    const double ship_input_downrange_km = std::max(0.0, in.ship_downrange_km);
    Input solve_in = in;
    double ship_opt_downrange_km = clampd(ship_input_downrange_km, 80.0, 2200.0);
    Stage1Out s1{};
    for (int it = 0; it < 5; ++it) {
        solve_in.ship_downrange_km = ship_opt_downrange_km;
        s1 = search_stage1(solve_in);
        const double iip_guess_km = stage1_iip_km(s1);
        const double target_ship_km = clampd(iip_guess_km, 80.0, 2200.0);
        if (std::abs(target_ship_km - ship_opt_downrange_km) < 0.5) {
            ship_opt_downrange_km = target_ship_km;
            break;
        }
        ship_opt_downrange_km = 0.5 * ship_opt_downrange_km + 0.5 * target_ship_km;
    }
    solve_in.ship_downrange_km = ship_opt_downrange_km;
    s1 = search_stage1(solve_in);

    const double sep_spd = std::hypot(s1.sep.vx, s1.sep.vz);
    const double sep_alt = s1.sep.z / 1000.0;

    const double s2full = in.s2_dry_kg + in.s2_prop_kg + in.payload_kg;
    double s1_sep_m = s1.sep.m - s2full;
    s1_sep_m = std::max(s1_sep_m, in.s1_dry_kg);

    const double tfall = stage1_fall_time_sec(s1);
    const double iip_km = stage1_iip_km(s1);
    const double iip_err = iip_km - ship_opt_downrange_km;

    const double dv_corr = std::min(1200.0, std::abs(iip_err) * 1.7);
    const double dv_entry = 170.0 + 0.03 * std::max(0.0, sep_spd - 1800.0);
    const double dv_land = 240.0 + 0.015 * std::max(0.0, sep_spd - 1700.0);
    const double dv_rec_req = dv_corr + dv_entry + dv_land;
    const double dv_rec_avail =
        (s1_sep_m > in.s1_dry_kg) ? in.s1_isp_s * kG0 * std::log(s1_sep_m / in.s1_dry_kg) : 0.0;
    const double rec_prop_need = prop_for_dv(s1_sep_m, dv_rec_req, in.s1_isp_s);
    const double rec_margin = s1.rem_prop - rec_prop_need;
    const bool rec_ok = rec_margin >= -1e-3;
    const double reserve_needed_ratio = (in.s1_prop_kg > 1e-9) ? (rec_prop_need / in.s1_prop_kg) : 0.0;
    const double reserve_actual_ratio = (in.s1_prop_kg > 1e-9) ? (s1.rem_prop / in.s1_prop_kg) : 0.0;

    const double rp = kRe + rp_km * 1000.0;
    const double ra = kRe + ra_km * 1000.0;
    const double a = 0.5 * (rp + ra);
    const double v_orbit = std::sqrt(kMu * (2.0 / rp - 1.0 / a));

    const double lat = deg2rad(in.lat_deg);
    const double inc = deg2rad(in.incl_deg);
    const double c_lat = std::cos(lat);
    const double vrot = kOmega * kRe * c_lat;
    bool direct_inc = true;
    double plane_penalty = 0.0;
    double assist = 0.0;
    double sin_az_launch = 0.0;
    if (std::abs(c_lat) > 1e-6) {
        double sin_az = std::cos(inc) / c_lat;
        if (sin_az < -1.0 || sin_az > 1.0) {
            direct_inc = false;
            const double min_i = std::abs(in.lat_deg);
            const double di = std::max(0.0, min_i - std::abs(in.incl_deg));
            plane_penalty = 2.0 * v_orbit * std::sin(0.5 * deg2rad(di));
            sin_az = clampd(sin_az, -1.0, 1.0);
        }
        sin_az_launch = sin_az;
        assist = vrot * std::max(0.0, sin_az);
    }
    const double launch_az_deg = std::asin(clampd(sin_az_launch, -1.0, 1.0)) * 180.0 / 3.14159265358979323846;
    const double launch_lon_deg = in.launch_lon_deg;
    double ship_lat_deg = in.lat_deg;
    double ship_lon_deg = launch_lon_deg;
    destination_from_course(
        in.lat_deg,
        launch_lon_deg,
        launch_az_deg,
        std::max(0.0, ship_opt_downrange_km),
        ship_lat_deg,
        ship_lon_deg);
    double iip_lat_deg = in.lat_deg;
    double iip_lon_deg = launch_lon_deg;
    destination_from_course(
        in.lat_deg,
        launch_lon_deg,
        launch_az_deg,
        std::max(0.0, iip_km),
        iip_lat_deg,
        iip_lon_deg);

    const double dv_total = v_orbit + in.losses_mps - assist + plane_penalty;
    const double dv_s2_req = std::max(0.0, dv_total - sep_spd) + 120.0;
    const double s2m0 = s2full;
    const double s2mf = in.s2_dry_kg + in.payload_kg;
    const double dv_s2_avail = in.s2_isp_s * kG0 * std::log(s2m0 / std::max(1.0, s2mf));
    const double s2_need = prop_for_dv(s2m0, dv_s2_req, in.s2_isp_s);
    const double s2_margin = in.s2_prop_kg - s2_need;
    const bool s2_ok = s2_margin >= -1e-3;

    const double mdot2 = (in.s2_thrust_kN * 1000.0) / std::max(1e-6, in.s2_isp_s * kG0);
    const double s2_burn = s2_need / std::max(1e-6, mdot2);

    // Upper-bound feasibility quick check (ideal rocket equation, no trajectory losses beyond dv_total).
    const double stack_m0 = in.s1_dry_kg + in.s1_prop_kg + s2full;
    const double stack_mf_reserve = in.s1_dry_kg + s2full + s1.rem_prop;
    const double stack_mf_noreserve = in.s1_dry_kg + s2full;
    const double dv_s1_upper_reserve =
        in.s1_isp_s * kG0 * std::log(stack_m0 / std::max(1.0, stack_mf_reserve));
    const double dv_s1_upper_noreserve =
        in.s1_isp_s * kG0 * std::log(stack_m0 / std::max(1.0, stack_mf_noreserve));
    const double dv_upper_reserve = dv_s1_upper_reserve + dv_s2_avail;
    const double dv_upper_noreserve = dv_s1_upper_noreserve + dv_s2_avail;
    const bool hard_impossible_with_reserve = dv_upper_reserve + 1e-6 < dv_total;
    const bool hard_impossible_even_noreserve = dv_upper_noreserve + 1e-6 < dv_total;

    const bool q_ok = s1.max_q <= in.q_limit_kpa * 1.05;
    p.ok = q_ok && rec_ok && s2_ok;
    p.status = p.ok ? L"FEASIBLE" : L"MARGINAL / NOT FEASIBLE";

    p.lines.push_back(L"[Status] " + p.status);
    p.lines.push_back(L"[Orbit] rp=" + fnum(rp_km, 1) + L" km, ra=" + fnum(ra_km, 1) + L" km, i=" + fnum(in.incl_deg, 2) + L" deg");
    p.lines.push_back(L"[Stage1 Optimization] auto-search burn and reserve split to maximize Stage1 recovery + Stage2 orbit margins under Max-Q.");
    p.lines.push_back(L"[Launch Site LL] lat=" + fnum(in.lat_deg, 6) + L", lon=" + fnum(launch_lon_deg, 6));
    p.lines.push_back(L"[Ascent 3DOF+Shooting] burn=" + fnum(s1.burn_s, 1) + L" s, sep_alt=" + fnum(sep_alt, 2) + L" km, sep_speed=" + fnum(sep_spd, 1) + L" m/s, maxQ=" + fnum(s1.max_q, 2) + L" kPa");
    if (!q_ok) p.lines.push_back(L"[Warning] Max-Q exceeded by " + fnum(s1.max_q - in.q_limit_kpa, 2) + L" kPa");
    p.lines.push_back(
        L"[Stage1 Recovery] IIP=" + fnum(iip_km, 1) +
        L" km, ship_opt=" + fnum(ship_opt_downrange_km, 1) +
        L" km, ship_input=" + fnum(ship_input_downrange_km, 1) +
        L" km, error=" + fnum(iip_err, 1) + L" km");
    p.lines.push_back(L"[IIP LL] lat=" + fnum(iip_lat_deg, 6) + L", lon=" + fnum(iip_lon_deg, 6));
    p.lines.push_back(L"[Recovery Ship LL] lat=" + fnum(ship_lat_deg, 6) + L", lon=" + fnum(ship_lon_deg, 6));
    p.lines.push_back(L"[Stage1 dV] req=" + fnum(dv_rec_req, 1) + L" m/s, avail=" + fnum(dv_rec_avail, 1) + L" m/s, fuel_margin=" + fnum(rec_margin, 1) + L" kg");
    p.lines.push_back(
        L"[Stage1 Reserve] required_for_recovery=" + fnum(rec_prop_need, 1) + L" kg (" +
        fnum(100.0 * reserve_needed_ratio, 2) + L"%), actual_remaining_after_ascent=" +
        fnum(s1.rem_prop, 1) + L" kg (" + fnum(100.0 * reserve_actual_ratio, 2) + L"%)");
    p.lines.push_back(L"[Stage2 dV] req=" + fnum(dv_s2_req, 1) + L" m/s, avail=" + fnum(dv_s2_avail, 1) + L" m/s, fuel_margin=" + fnum(s2_margin, 1) + L" kg");
    p.lines.push_back(
        L"[QuickCheck dV Upper] req_no_recovery=" + fnum(dv_total, 1) +
        L" m/s, upper(reserve on)=" + fnum(dv_upper_reserve, 1) +
        L" m/s, upper(no reserve)=" + fnum(dv_upper_noreserve, 1) + L" m/s");
    if (hard_impossible_even_noreserve) {
        p.lines.push_back(L"[QuickCheck] Physically impossible for this vehicle/propellant even with zero stage1 reserve.");
    } else if (hard_impossible_with_reserve) {
        p.lines.push_back(L"[QuickCheck] Infeasible under current stage1 reserve; reducing reserve or increasing propellant may help.");
    }
    if (!direct_inc) p.lines.push_back(L"[Note] Inclination below launch latitude, added plane-change penalty " + fnum(plane_penalty, 1) + L" m/s");
    p.lines.push_back(L"[Stage2 Burn] " + fnum(s2_burn, 1) + L" s, Earth-rotation assist=" + fnum(assist, 1) + L" m/s");
    p.lines.push_back(L"[Timeline] " + ftime(0.0) + L" Liftoff");
    p.lines.push_back(L"[Timeline] " + ftime(s1.t_max_q) + L" Max-Q");
    p.lines.push_back(L"[Timeline] " + ftime(s1.burn_s) + L" Stage Separation");
    p.lines.push_back(L"[Timeline] " + ftime(s1.burn_s + 30.0) + L" Recovery Correction Burn");
    p.lines.push_back(L"[Timeline] " + ftime(s1.burn_s + std::max(200.0, tfall - 120.0)) + L" Entry Burn");
    p.lines.push_back(L"[Timeline] " + ftime(s1.burn_s + std::max(300.0, tfall - 30.0)) + L" Landing Burn");
    p.lines.push_back(L"[Timeline] " + ftime(s1.burn_s + tfall) + L" Droneship Touchdown");
    p.lines.push_back(L"[Timeline] " + ftime(s1.burn_s + 6.0 + s2_burn) + L" Stage2 SECO");

    Series a1;
    a1.name = L"Ascent (Stage1)";
    a1.color = RGB(41, 128, 185);
    for (const SimPt& pt : s1.traj) a1.pts.push_back({pt.x_km, pt.z_km});
    p.profile_series.push_back(std::move(a1));

    const double sx = s1.sep.x / 1000.0;
    const double sz = s1.sep.z / 1000.0;
    Series rec;
    rec.name = L"Sea Recovery";
    rec.color = RGB(39, 174, 96);
    rec.pts = bezier(
        {sx, sz},
        {sx + 0.35 * (iip_km - sx), sz + 25.0},
        {ship_opt_downrange_km + 0.30 * (iip_km - ship_opt_downrange_km), 15.0},
        {ship_opt_downrange_km, 0.0},
        72);
    p.profile_series.push_back(std::move(rec));

    const double stage2_range = std::max(450.0, 0.5 * (sep_spd + v_orbit) * s2_burn / 1000.0);
    const double stage2_end_downrange_km = sx + stage2_range;
    Series orb;
    orb.name = L"Orbit Injection (Stage2)";
    orb.color = RGB(192, 57, 43);
    orb.pts = bezier(
        {sx, sz},
        {sx + 0.22 * stage2_range, sz + 55.0},
        {sx + 0.76 * stage2_range, rp_km + 20.0},
        {stage2_end_downrange_km, rp_km},
        72);
    p.profile_series.push_back(std::move(orb));

    p.launch_lat_deg = in.lat_deg;
    p.launch_lon_deg = launch_lon_deg;
    p.ship_lat_deg = ship_lat_deg;
    p.ship_lon_deg = ship_lon_deg;

    // Center camera between launch and droneship for a full-Earth context.
    p.view_lat_deg = 0.5 * (p.launch_lat_deg + p.ship_lat_deg);
    {
        const double lon_mid = p.launch_lon_deg + 0.5 * (p.ship_lon_deg - p.launch_lon_deg);
        p.view_lon_deg = wrap_lon_deg(lon_mid);
    }

    auto profile_to_globe = [&](const Series& src) {
        GlobeSeries gs;
        gs.name = src.name;
        gs.color = src.color;
        gs.pts.reserve(src.pts.size());
        for (const PlotPt& q : src.pts) {
            GlobePt gp;
            destination_from_course(
                in.lat_deg,
                p.launch_lon_deg,
                launch_az_deg,
                std::max(0.0, q.x_km),
                gp.lat_deg,
                gp.lon_deg);
            gp.alt_km = std::max(0.0, q.y_km);
            gs.pts.push_back(gp);
        }
        return gs;
    };

    for (const Series& s : p.profile_series) {
        p.globe_series.push_back(profile_to_globe(s));
    }

    // Add post-SECO orbital track on the whole-Earth view.
    GlobePt insertion_gp{};
    destination_from_course(
        in.lat_deg,
        p.launch_lon_deg,
        launch_az_deg,
        std::max(0.0, stage2_end_downrange_km),
        insertion_gp.lat_deg,
        insertion_gp.lon_deg);
    insertion_gp.alt_km = rp_km;

    const Vec3 rhat = normalize3(ecef_from_geo(insertion_gp.lat_deg, insertion_gp.lon_deg, 0.0));
    const Vec3 east = normalize3({-std::sin(deg2rad(insertion_gp.lon_deg)), std::cos(deg2rad(insertion_gp.lon_deg)), 0.0});
    const Vec3 north = normalize3({
        -std::sin(deg2rad(insertion_gp.lat_deg)) * std::cos(deg2rad(insertion_gp.lon_deg)),
        -std::sin(deg2rad(insertion_gp.lat_deg)) * std::sin(deg2rad(insertion_gp.lon_deg)),
        std::cos(deg2rad(insertion_gp.lat_deg))});

    const double az_rad = deg2rad(launch_az_deg);
    const Vec3 vref = normalize3({
        north.x * std::cos(az_rad) + east.x * std::sin(az_rad),
        north.y * std::cos(az_rad) + east.y * std::sin(az_rad),
        north.z * std::cos(az_rad) + east.z * std::sin(az_rad),
    });

    Vec3 h_orb = normalize3(cross3(rhat, vref));
    const double hz_target = std::cos(deg2rad(in.incl_deg));
    const double A = rhat.x;
    const double B = rhat.y;
    const double C = -hz_target * rhat.z;
    const double norm_u2 = A * A + B * B;
    const double s2 = std::max(0.0, 1.0 - hz_target * hz_target);
    if (norm_u2 > 1e-10) {
        const double p0x = (C / norm_u2) * A;
        const double p0y = (C / norm_u2) * B;
        const double d2 = p0x * p0x + p0y * p0y;
        if (d2 <= s2 + 1e-8) {
            const double t = std::sqrt(std::max(0.0, s2 - d2));
            const double invn = 1.0 / std::sqrt(norm_u2);
            const double vx = -B * invn;
            const double vy = A * invn;

            const Vec3 h1 = normalize3({p0x + t * vx, p0y + t * vy, hz_target});
            const Vec3 h2 = normalize3({p0x - t * vx, p0y - t * vy, hz_target});

            const Vec3 v1 = normalize3(cross3(h1, rhat));
            const Vec3 v2 = normalize3(cross3(h2, rhat));
            const double az1 = azimuth_from_north_east(dot3(v1, north), dot3(v1, east));
            const double az2 = azimuth_from_north_east(dot3(v2, north), dot3(v2, east));

            h_orb = (ang_diff_deg(az1, launch_az_deg) <= ang_diff_deg(az2, launch_az_deg)) ? h1 : h2;
        }
    }
    h_orb = normalize3(h_orb);

    const double rp_orb = kRe / 1000.0 + rp_km;
    const double ra_orb = kRe / 1000.0 + ra_km;
    const double a_orb = 0.5 * (rp_orb + ra_orb);
    const double e_orb = (ra_orb - rp_orb) / std::max(1e-9, (ra_orb + rp_orb));
    const double p_orb = a_orb * (1.0 - e_orb * e_orb);

    const Vec3 u_orb = rhat;
    Vec3 w_orb = normalize3(cross3(h_orb, u_orb));
    if (dot3(w_orb, w_orb) < 1e-10) {
        w_orb = normalize3(cross3({0.0, 0.0, 1.0}, u_orb));
    }

    GlobeSeries post_orbit;
    post_orbit.name = L"Post-Insertion Orbit";
    post_orbit.color = RGB(243, 156, 18);
    const int n_orbit = 540;
    post_orbit.pts.reserve(static_cast<size_t>(n_orbit + 1));
    for (int i = 0; i <= n_orbit; ++i) {
        const double nu = (2.0 * 3.14159265358979323846 * static_cast<double>(i)) / static_cast<double>(n_orbit);
        const double rmag = p_orb / std::max(1e-8, (1.0 + e_orb * std::cos(nu)));
        Vec3 q{
            rmag * (std::cos(nu) * u_orb.x + std::sin(nu) * w_orb.x),
            rmag * (std::cos(nu) * u_orb.y + std::sin(nu) * w_orb.y),
            rmag * (std::cos(nu) * u_orb.z + std::sin(nu) * w_orb.z),
        };
        const double qn = std::sqrt(dot3(q, q));
        GlobePt gp{};
        gp.lat_deg = std::asin(clampd(q.z / std::max(1e-9, qn), -1.0, 1.0)) * 180.0 / 3.14159265358979323846;
        gp.lon_deg = wrap_lon_deg(std::atan2(q.y, q.x) * 180.0 / 3.14159265358979323846);
        gp.alt_km = std::max(0.0, qn - (kRe / 1000.0));
        post_orbit.pts.push_back(gp);
    }
    p.globe_series.push_back(std::move(post_orbit));

    return p;
}

std::vector<Field> make_fields(const Input& d) {
    int id = kFieldBase;
    return {
        {id++, L"Perigee Altitude", L"km", d.perigee_km, &Input::perigee_km},
        {id++, L"Apogee Altitude", L"km", d.apogee_km, &Input::apogee_km},
        {id++, L"Inclination", L"deg", d.incl_deg, &Input::incl_deg},
    };
}

App* app(HWND hwnd) {
    return reinterpret_cast<App*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
}

void set_font(HWND h, HFONT f) {
    if (h && f) SendMessageW(h, WM_SETFONT, reinterpret_cast<WPARAM>(f), TRUE);
}

void layout(HWND hwnd, App& a) {
    RECT rc{};
    GetClientRect(hwnd, &rc);
    const int left_w = 430;
    const int x0 = 12;
    int y = 12;
    const int row_h = 22;
    const int gap = 4;
    const int lw = 210;
    const int ew = 116;
    const int uw = 64;

    for (Field& f : a.fields) {
        MoveWindow(f.h_label, x0, y + 2, lw, row_h, TRUE);
        MoveWindow(f.h_edit, x0 + lw + 8, y, ew, row_h, TRUE);
        MoveWindow(f.h_unit, x0 + lw + ew + 14, y + 2, uw, row_h, TRUE);
        y += row_h + gap;
    }
    MoveWindow(a.btn, x0, y + 8, left_w - 24, 30, TRUE);
    MoveWindow(a.btn_max_payload, x0, y + 44, left_w - 24, 30, TRUE);

    const int rc_right = static_cast<int>(rc.right);
    const int rc_bottom = static_cast<int>(rc.bottom);
    const int pl = left_w + 16;
    const int pt = 14;
    const int pr = rc_right - 14;
    const int pb = std::max(pt + 240, rc_bottom - 230);
    a.plot = {pl, pt, pr, pb};
    RECT left{};
    RECT right{};
    split_plot_rects(a.plot, left, right);
    a.globe_panel = right;

    MoveWindow(a.list, pl, pb + 10, std::max(50, pr - pl), std::max(50, rc_bottom - (pb + 24)), TRUE);
}

bool read_inputs(const App& a, Input& in, std::wstring& err) {
    in = a.base_input;
    for (const Field& f : a.fields) {
        wchar_t buf[128]{};
        GetWindowTextW(f.h_edit, buf, static_cast<int>(std::size(buf)));
        double v = 0.0;
        if (!parse_num(buf, v)) {
            err = std::wstring(L"Invalid numeric input: ") + f.label;
            return false;
        }
        in.*(f.member) = v;
    }
    return true;
}

void refresh_list(const App& a) {
    SendMessageW(a.list, LB_RESETCONTENT, 0, 0);
    for (const std::wstring& s : a.plan.lines) {
        SendMessageW(a.list, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(s.c_str()));
    }
}

void prepend_vehicle_config_note(App& a) {
    if (a.has_vehicle_config) {
        a.plan.lines.insert(
            a.plan.lines.begin(),
            L"[Vehicle Config] Loaded from: " + a.vehicle_config_path);
    } else if (!a.vehicle_config_error.empty()) {
        a.plan.lines.insert(
            a.plan.lines.begin(),
            L"[Vehicle Config] Load failed: " + a.vehicle_config_error);
    } else {
        a.plan.lines.insert(
            a.plan.lines.begin(),
            L"[Vehicle Config] Not provided; using internal defaults.");
    }
}

void run_plan(HWND hwnd, App& a) {
    Input in{};
    std::wstring err;
    if (!read_inputs(a, in, err)) {
        MessageBoxW(hwnd, err.c_str(), L"Input Error", MB_OK | MB_ICONWARNING);
        return;
    }
    a.plan = build_plan(in);
    prepend_vehicle_config_note(a);
    if (!a.view_initialized) {
        a.view_lat_deg = a.plan.view_lat_deg;
        a.view_lon_deg = a.plan.view_lon_deg;
        a.view_initialized = true;
    }
    refresh_list(a);
    InvalidateRect(hwnd, &a.plot, TRUE);
}

void run_max_payload(HWND hwnd, App& a) {
    Input in{};
    std::wstring err;
    if (!read_inputs(a, in, err)) {
        MessageBoxW(hwnd, err.c_str(), L"Input Error", MB_OK | MB_ICONWARNING);
        return;
    }

    auto eval_payload = [&](double payload_kg, Plan* out_plan) {
        Input test = in;
        test.payload_kg = std::max(0.0, payload_kg);
        Plan p = build_plan(test);
        const bool ok = p.ok;
        if (out_plan) *out_plan = std::move(p);
        return ok;
    };

    Plan plan_zero{};
    if (!eval_payload(0.0, &plan_zero)) {
        a.plan = std::move(plan_zero);
        a.plan.lines.insert(
            a.plan.lines.begin(),
            L"[Payload Capability] Max payload = N/A");
        a.plan.lines.insert(
            a.plan.lines.begin() + 1,
            L"[Payload Capability] Infeasible even at payload = 0 kg for current orbit/vehicle settings.");
        prepend_vehicle_config_note(a);
        refresh_list(a);
        InvalidateRect(hwnd, &a.plot, TRUE);
        return;
    }

    double lo = 0.0;
    double hi = std::max(5000.0, in.payload_kg);
    Plan best_plan = plan_zero;
    Plan probe{};

    // Coarse expansion to find an infeasible upper bound.
    for (int i = 0; i < 10; ++i) {
        if (eval_payload(hi, &probe)) {
            lo = hi;
            best_plan = std::move(probe);
            hi *= 1.6;
        } else {
            break;
        }
    }
    if (hi <= lo + 1.0) hi = lo + 1000.0;

    // Binary refinement.
    for (int i = 0; i < 22; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (eval_payload(mid, &probe)) {
            lo = mid;
            best_plan = std::move(probe);
        } else {
            hi = mid;
        }
    }

    a.plan = std::move(best_plan);
    a.plan.lines.insert(
        a.plan.lines.begin(),
        L"[Payload Capability] Max payload at current orbit ~ " + fnum(lo, 1) + L" kg");
    a.plan.lines.insert(
        a.plan.lines.begin() + 1,
        L"[Payload Capability] Feasible interval: [" + fnum(lo, 1) + L", " + fnum(hi, 1) + L"] kg");
    prepend_vehicle_config_note(a);

    if (!a.view_initialized) {
        a.view_lat_deg = a.plan.view_lat_deg;
        a.view_lon_deg = a.plan.view_lon_deg;
        a.view_initialized = true;
    }
    refresh_list(a);
    InvalidateRect(hwnd, &a.plot, TRUE);
}

void draw_panel_frame(HDC hdc, const RECT& outer) {
    HBRUSH bg = CreateSolidBrush(RGB(252, 252, 252));
    FillRect(hdc, &outer, bg);
    DeleteObject(bg);

    HPEN bd = CreatePen(PS_SOLID, 1, RGB(170, 170, 170));
    HPEN oldp = reinterpret_cast<HPEN>(SelectObject(hdc, bd));
    HBRUSH oldb = reinterpret_cast<HBRUSH>(SelectObject(hdc, GetStockObject(HOLLOW_BRUSH)));
    Rectangle(hdc, outer.left, outer.top, outer.right, outer.bottom);
    SelectObject(hdc, oldb);
    SelectObject(hdc, oldp);
    DeleteObject(bd);
}

void split_plot_rects(const RECT& outer, RECT& left, RECT& right) {
    const int split = outer.left + static_cast<int>((outer.right - outer.left) * 0.50);
    left = outer;
    right = outer;
    left.right = split - 4;
    right.left = split + 4;
}

void draw_profile_panel(HDC hdc, const RECT& outer, const Plan& p) {
    draw_panel_frame(hdc, outer);

    if (p.profile_series.empty()) {
        const wchar_t* msg = L"Press \"Plan Mission\" to generate trajectory.";
        TextOutW(hdc, outer.left + 16, outer.top + 16, msg, lstrlenW(msg));
        return;
    }

    double xmin = 0.0;
    double xmax = 1.0;
    double ymin = 0.0;
    double ymax = 1.0;
    bool first = true;
    for (const Series& s : p.profile_series) {
        for (const PlotPt& q : s.pts) {
            if (first) {
                xmin = xmax = q.x_km;
                ymin = ymax = q.y_km;
                first = false;
            } else {
                xmin = std::min(xmin, q.x_km);
                xmax = std::max(xmax, q.x_km);
                ymin = std::min(ymin, q.y_km);
                ymax = std::max(ymax, q.y_km);
            }
        }
    }
    xmin = std::min(0.0, xmin);
    ymin = std::min(0.0, ymin);
    if (xmax <= xmin + 1e-6) xmax = xmin + 1.0;
    if (ymax <= ymin + 1e-6) ymax = ymin + 1.0;

    RECT in = outer;
    in.left += 54;
    in.right -= 20;
    in.top += 22;
    in.bottom -= 38;
    if (in.right <= in.left + 10 || in.bottom <= in.top + 10) return;

    const wchar_t* title = L"Trajectory Profile";
    TextOutW(hdc, outer.left + 10, outer.top + 5, title, lstrlenW(title));

    HPEN ax = CreatePen(PS_SOLID, 1, RGB(110, 110, 110));
    HPEN oldp = reinterpret_cast<HPEN>(SelectObject(hdc, ax));
    MoveToEx(hdc, in.left, in.bottom, nullptr);
    LineTo(hdc, in.right, in.bottom);
    MoveToEx(hdc, in.left, in.top, nullptr);
    LineTo(hdc, in.left, in.bottom);
    SelectObject(hdc, oldp);
    DeleteObject(ax);

    SetBkMode(hdc, TRANSPARENT);
    for (int i = 0; i <= 5; ++i) {
        const double u = static_cast<double>(i) / 5.0;
        const int x = in.left + static_cast<int>(std::lround((in.right - in.left) * u));
        MoveToEx(hdc, x, in.bottom, nullptr);
        LineTo(hdc, x, in.bottom + 4);
        const std::wstring t = fnum(xmin + (xmax - xmin) * u, 0);
        TextOutW(hdc, x - 18, in.bottom + 8, t.c_str(), static_cast<int>(t.size()));
    }
    for (int i = 0; i <= 5; ++i) {
        const double u = static_cast<double>(i) / 5.0;
        const int y = in.bottom - static_cast<int>(std::lround((in.bottom - in.top) * u));
        MoveToEx(hdc, in.left - 4, y, nullptr);
        LineTo(hdc, in.left, y);
        const std::wstring t = fnum(ymin + (ymax - ymin) * u, 0);
        TextOutW(hdc, in.left - 46, y - 7, t.c_str(), static_cast<int>(t.size()));
    }
    const wchar_t* xl = L"Downrange (km)";
    const wchar_t* yl = L"Altitude (km)";
    TextOutW(hdc, (in.left + in.right) / 2 - 48, in.bottom + 24, xl, lstrlenW(xl));
    TextOutW(hdc, outer.left + 6, outer.top + 2, yl, lstrlenW(yl));

    auto map_pt = [&](const PlotPt& q) {
        POINT p2{};
        const double nx = (q.x_km - xmin) / (xmax - xmin);
        const double ny = (q.y_km - ymin) / (ymax - ymin);
        p2.x = in.left + static_cast<int>(std::lround(nx * (in.right - in.left)));
        p2.y = in.bottom - static_cast<int>(std::lround(ny * (in.bottom - in.top)));
        return p2;
    };

    int ly = outer.top + 8;
    const int lx = outer.right - 210;
    for (const Series& s : p.profile_series) {
        if (s.pts.size() < 2) continue;
        std::vector<POINT> pts;
        pts.reserve(s.pts.size());
        for (const PlotPt& q : s.pts) pts.push_back(map_pt(q));
        HPEN pen = CreatePen(PS_SOLID, 2, s.color);
        oldp = reinterpret_cast<HPEN>(SelectObject(hdc, pen));
        Polyline(hdc, pts.data(), static_cast<int>(pts.size()));
        SelectObject(hdc, oldp);
        DeleteObject(pen);

        HPEN lpen = CreatePen(PS_SOLID, 2, s.color);
        oldp = reinterpret_cast<HPEN>(SelectObject(hdc, lpen));
        MoveToEx(hdc, lx, ly + 7, nullptr);
        LineTo(hdc, lx + 24, ly + 7);
        SelectObject(hdc, oldp);
        DeleteObject(lpen);
        TextOutW(hdc, lx + 30, ly, s.name.c_str(), static_cast<int>(s.name.size()));
        ly += 18;
    }
}

void draw_globe_panel(HDC hdc, const RECT& outer, const Plan& p, double view_lat_deg, double view_lon_deg) {
    draw_panel_frame(hdc, outer);

    if (p.globe_series.empty()) {
        const wchar_t* msg = L"3D globe view will appear after planning.";
        TextOutW(hdc, outer.left + 16, outer.top + 16, msg, lstrlenW(msg));
        return;
    }

    const wchar_t* title = L"3D Earth View (Orthographic)";
    TextOutW(hdc, outer.left + 10, outer.top + 5, title, lstrlenW(title));
    const wchar_t* hint = L"Drag with left mouse button to rotate";
    TextOutW(hdc, outer.left + 220, outer.top + 5, hint, lstrlenW(hint));

    RECT in = outer;
    in.left += 14;
    in.right -= 14;
    in.top += 24;
    in.bottom -= 12;
    const int w = in.right - in.left;
    const int h = in.bottom - in.top;
    if (w <= 60 || h <= 60) return;

    const int cx = in.left + w / 2;
    const int cy = in.top + h / 2;
    const int radius_px = std::max(10, std::min(w, h) / 2 - 12);

    const Vec3 view_n = normalize3(ecef_from_geo(view_lat_deg, view_lon_deg, 0.0));
    const double view_lon = deg2rad(view_lon_deg);
    const double view_lat = deg2rad(view_lat_deg);
    const Vec3 view_u = normalize3({-std::sin(view_lon), std::cos(view_lon), 0.0});
    const Vec3 view_v = normalize3({
        -std::sin(view_lat) * std::cos(view_lon),
        -std::sin(view_lat) * std::sin(view_lon),
        std::cos(view_lat)});

    auto project = [&](const Vec3& q, POINT& out, double& depth) {
        depth = dot3(q, view_n);
        out.x = cx + static_cast<int>(std::lround(dot3(q, view_u) * radius_px));
        out.y = cy - static_cast<int>(std::lround(dot3(q, view_v) * radius_px));
    };

    HBRUSH ocean = CreateSolidBrush(RGB(224, 239, 252));
    HBRUSH oldb = reinterpret_cast<HBRUSH>(SelectObject(hdc, ocean));
    HPEN globe_border = CreatePen(PS_SOLID, 1, RGB(120, 150, 180));
    HPEN oldp = reinterpret_cast<HPEN>(SelectObject(hdc, globe_border));
    Ellipse(hdc, cx - radius_px, cy - radius_px, cx + radius_px, cy + radius_px);
    SelectObject(hdc, oldp);
    SelectObject(hdc, oldb);
    DeleteObject(globe_border);
    DeleteObject(ocean);

    auto draw_geo_line = [&](bool is_lat, double fixed_deg, COLORREF color, int width) {
        HPEN pen = CreatePen(PS_SOLID, width, color);
        HPEN prev_pen = reinterpret_cast<HPEN>(SelectObject(hdc, pen));
        bool prev_vis = false;
        POINT prev_pt{};
        for (int k = -180; k <= 180; k += 3) {
            double lat_deg = 0.0;
            double lon_deg = 0.0;
            if (is_lat) {
                lat_deg = fixed_deg;
                lon_deg = static_cast<double>(k);
            } else {
                lat_deg = static_cast<double>(k) * 0.5;
                lon_deg = fixed_deg;
            }
            const Vec3 q = ecef_from_geo(lat_deg, lon_deg, 0.0);
            POINT pt{};
            double depth = 0.0;
            project(q, pt, depth);
            const bool vis = depth >= 0.0;
            if (vis && prev_vis) {
                MoveToEx(hdc, prev_pt.x, prev_pt.y, nullptr);
                LineTo(hdc, pt.x, pt.y);
            }
            prev_vis = vis;
            prev_pt = pt;
        }
        SelectObject(hdc, prev_pen);
        DeleteObject(pen);
    };

    for (int lat_deg = -60; lat_deg <= 60; lat_deg += 30) {
        draw_geo_line(true, static_cast<double>(lat_deg), RGB(205, 215, 225), 1);
    }
    for (int lon_deg = -150; lon_deg <= 180; lon_deg += 30) {
        draw_geo_line(false, static_cast<double>(lon_deg), RGB(205, 215, 225), 1);
    }

    for (const GlobeSeries& s : p.globe_series) {
        if (s.pts.size() < 2) continue;
        HPEN pen = CreatePen(PS_SOLID, 2, s.color);
        HPEN prev_pen = reinterpret_cast<HPEN>(SelectObject(hdc, pen));
        for (size_t i = 1; i < s.pts.size(); ++i) {
            const Vec3 q1 = ecef_from_geo(s.pts[i - 1].lat_deg, s.pts[i - 1].lon_deg, s.pts[i - 1].alt_km);
            const Vec3 q2 = ecef_from_geo(s.pts[i].lat_deg, s.pts[i].lon_deg, s.pts[i].alt_km);
            POINT p1{};
            POINT p2{};
            double d1 = 0.0;
            double d2 = 0.0;
            project(q1, p1, d1);
            project(q2, p2, d2);
            if (d1 >= 0.0 && d2 >= 0.0) {
                MoveToEx(hdc, p1.x, p1.y, nullptr);
                LineTo(hdc, p2.x, p2.y);
            }
        }
        SelectObject(hdc, prev_pen);
        DeleteObject(pen);
    }

    auto draw_site = [&](double lat_deg, double lon_deg, COLORREF color, const wchar_t* tag) {
        const Vec3 q = ecef_from_geo(lat_deg, lon_deg, 0.0);
        POINT pt{};
        double d = 0.0;
        project(q, pt, d);
        if (d < 0.0) return;
        HBRUSH brush = CreateSolidBrush(color);
        HBRUSH old_br = reinterpret_cast<HBRUSH>(SelectObject(hdc, brush));
        HPEN pen = CreatePen(PS_SOLID, 1, color);
        HPEN old_pen = reinterpret_cast<HPEN>(SelectObject(hdc, pen));
        Ellipse(hdc, pt.x - 4, pt.y - 4, pt.x + 4, pt.y + 4);
        SelectObject(hdc, old_pen);
        SelectObject(hdc, old_br);
        DeleteObject(brush);
        DeleteObject(pen);
        TextOutW(hdc, pt.x + 6, pt.y - 8, tag, lstrlenW(tag));
    };
    draw_site(p.launch_lat_deg, p.launch_lon_deg, RGB(52, 73, 94), L"Launch");
    draw_site(p.ship_lat_deg, p.ship_lon_deg, RGB(22, 160, 133), L"Droneship");

    int ly = outer.top + 8;
    const int lx = outer.left + 12;
    for (const GlobeSeries& s : p.globe_series) {
        HPEN lpen = CreatePen(PS_SOLID, 2, s.color);
        HPEN prev_pen = reinterpret_cast<HPEN>(SelectObject(hdc, lpen));
        MoveToEx(hdc, lx, ly + 7, nullptr);
        LineTo(hdc, lx + 24, ly + 7);
        SelectObject(hdc, prev_pen);
        DeleteObject(lpen);
        TextOutW(hdc, lx + 30, ly, s.name.c_str(), static_cast<int>(s.name.size()));
        ly += 18;
    }
}

void draw_plot(HDC hdc, const RECT& outer, const Plan& p, double view_lat_deg, double view_lon_deg) {
    HBRUSH bg = CreateSolidBrush(RGB(245, 247, 250));
    FillRect(hdc, &outer, bg);
    DeleteObject(bg);

    RECT left{};
    RECT right{};
    split_plot_rects(outer, left, right);

    draw_profile_panel(hdc, left, p);
    draw_globe_panel(hdc, right, p, view_lat_deg, view_lon_deg);
}

LRESULT CALLBACK wndproc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_NCCREATE: {
            auto* a = new App();
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(a));
            return TRUE;
        }
        case WM_CREATE: {
            App* a = app(hwnd);
            if (!a) return -1;
            a->font = reinterpret_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));
            a->base_input = Input{};
            a->has_vehicle_config = g_startup_vehicle.has_config;
            a->vehicle_config_path = g_startup_vehicle.config_path;
            a->vehicle_config_error = g_startup_vehicle.error;
            if (g_startup_vehicle.has_config) {
                a->base_input = g_startup_vehicle.input;
            }
            a->fields = make_fields(a->base_input);
            for (Field& f : a->fields) {
                f.h_label = CreateWindowExW(0, L"STATIC", f.label, WS_CHILD | WS_VISIBLE, 0, 0, 100, 20, hwnd, nullptr, nullptr, nullptr);
                f.h_edit = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"", WS_CHILD | WS_VISIBLE | ES_AUTOHSCROLL, 0, 0, 100, 20, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(f.id)), nullptr, nullptr);
                f.h_unit = CreateWindowExW(0, L"STATIC", f.unit, WS_CHILD | WS_VISIBLE, 0, 0, 80, 20, hwnd, nullptr, nullptr, nullptr);
                const std::wstring init = fnum(f.dval, std::abs(f.dval) < 1.0 ? 3 : 1);
                SetWindowTextW(f.h_edit, init.c_str());
                set_font(f.h_label, a->font);
                set_font(f.h_edit, a->font);
                set_font(f.h_unit, a->font);
            }
            a->btn = CreateWindowExW(0, L"BUTTON", L"Plan Mission", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 100, 30, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnPlan)), nullptr, nullptr);
            a->btn_max_payload = CreateWindowExW(0, L"BUTTON", L"Compute Max Payload", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 100, 30, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnMaxPayload)), nullptr, nullptr);
            a->list = CreateWindowExW(WS_EX_CLIENTEDGE, L"LISTBOX", L"", WS_CHILD | WS_VISIBLE | WS_VSCROLL | LBS_NOINTEGRALHEIGHT, 0, 0, 200, 100, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kList)), nullptr, nullptr);
            set_font(a->btn, a->font);
            set_font(a->btn_max_payload, a->font);
            set_font(a->list, a->font);
            layout(hwnd, *a);
            run_plan(hwnd, *a);
            return 0;
        }
        case WM_SIZE: {
            App* a = app(hwnd);
            if (a) {
                layout(hwnd, *a);
                InvalidateRect(hwnd, nullptr, TRUE);
            }
            return 0;
        }
        case WM_COMMAND: {
            App* a = app(hwnd);
            if (a && LOWORD(wp) == kBtnPlan && HIWORD(wp) == BN_CLICKED) {
                run_plan(hwnd, *a);
                return 0;
            }
            if (a && LOWORD(wp) == kBtnMaxPayload && HIWORD(wp) == BN_CLICKED) {
                run_max_payload(hwnd, *a);
                return 0;
            }
            return 0;
        }
        case WM_LBUTTONDOWN: {
            App* a = app(hwnd);
            if (!a) return 0;
            POINT pt{GET_X_LPARAM(lp), GET_Y_LPARAM(lp)};
            if (PtInRect(&a->globe_panel, pt)) {
                a->dragging_globe = true;
                a->last_mouse = pt;
                SetCapture(hwnd);
                return 0;
            }
            return 0;
        }
        case WM_MOUSEMOVE: {
            App* a = app(hwnd);
            if (!a || !a->dragging_globe) return 0;
            POINT pt{GET_X_LPARAM(lp), GET_Y_LPARAM(lp)};
            const int dx = pt.x - a->last_mouse.x;
            const int dy = pt.y - a->last_mouse.y;
            a->last_mouse = pt;

            a->view_lon_deg = wrap_lon_deg(a->view_lon_deg - 0.35 * static_cast<double>(dx));
            a->view_lat_deg = clampd(a->view_lat_deg + 0.25 * static_cast<double>(dy), -89.0, 89.0);
            InvalidateRect(hwnd, &a->plot, FALSE);
            return 0;
        }
        case WM_LBUTTONUP: {
            App* a = app(hwnd);
            if (a && a->dragging_globe) {
                a->dragging_globe = false;
                if (GetCapture() == hwnd) ReleaseCapture();
            }
            return 0;
        }
        case WM_CAPTURECHANGED: {
            App* a = app(hwnd);
            if (a) a->dragging_globe = false;
            return 0;
        }
        case WM_PAINT: {
            App* a = app(hwnd);
            PAINTSTRUCT ps{};
            HDC hdc = BeginPaint(hwnd, &ps);
            if (a) draw_plot(hdc, a->plot, a->plan, a->view_lat_deg, a->view_lon_deg);
            EndPaint(hwnd, &ps);
            return 0;
        }
        case WM_DESTROY: {
            App* a = app(hwnd);
            delete a;
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, 0);
            PostQuitMessage(0);
            return 0;
        }
        default:
            return DefWindowProcW(hwnd, msg, wp, lp);
    }
}

} // namespace

int WINAPI wWinMain(HINSTANCE hi, HINSTANCE, PWSTR, int nshow) {
    {
        int argc = 0;
        LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
        if (argv) {
            std::wstring cfg_path;
            for (int i = 1; i < argc; ++i) {
                const std::wstring arg = argv[i] ? argv[i] : L"";
                if (arg == L"--vehicle-config" && i + 1 < argc) {
                    cfg_path = argv[++i];
                } else if (arg.rfind(L"--vehicle-config=", 0) == 0) {
                    cfg_path = arg.substr(std::wstring(L"--vehicle-config=").size());
                }
            }
            if (!cfg_path.empty()) {
                Input loaded{};
                std::wstring err;
                if (load_vehicle_config(cfg_path, loaded, err)) {
                    g_startup_vehicle.has_config = true;
                    g_startup_vehicle.config_path = cfg_path;
                    g_startup_vehicle.input = loaded;
                    g_startup_vehicle.error.clear();
                } else {
                    g_startup_vehicle.has_config = false;
                    g_startup_vehicle.config_path = cfg_path;
                    g_startup_vehicle.error = err;
                }
            }
            LocalFree(argv);
        }
    }

    INITCOMMONCONTROLSEX icc{};
    icc.dwSize = sizeof(icc);
    icc.dwICC = ICC_STANDARD_CLASSES;
    InitCommonControlsEx(&icc);

    const wchar_t* cls = L"Falcon9PlannerWinClass";
    WNDCLASSW wc{};
    wc.lpfnWndProc = wndproc;
    wc.hInstance = hi;
    wc.lpszClassName = cls;
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    if (!RegisterClassW(&wc)) return 1;

    HWND hwnd = CreateWindowExW(
        0,
        cls,
        L"Falcon9 GUI Planner (Launch -> Stage1 Sea Recovery -> Stage2 Orbit)",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        1360,
        860,
        nullptr,
        nullptr,
        hi,
        nullptr);
    if (!hwnd) return 1;

    ShowWindow(hwnd, nshow);
    UpdateWindow(hwnd);

    MSG msg{};
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return static_cast<int>(msg.wParam);
}
