#define NOMINMAX
#include <windows.h>
#include <commctrl.h>
#include <windowsx.h>

#include "planner_mission.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cwchar>
#include <cwctype>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <shellapi.h>

namespace {

using falcon9::GlobePt;
using falcon9::GlobeSeries;
using falcon9::MissionRequest;
using falcon9::MissionResult;
using falcon9::PlotPt;
using falcon9::SeparationCandidate;
using falcon9::Series;
using falcon9::Vec3;

constexpr int kBtnPlan = 1001;
constexpr int kList = 1002;
constexpr int kBtnMaxPayload = 1003;
constexpr int kBtnImportDefaultCfg = 1004;
constexpr int kBtnPlanForce = 1005;
constexpr int kBtnPlanBurnoutNoRecovery = 1006;
constexpr int kBtnPrevCandidate = 1007;
constexpr int kBtnNextCandidate = 1008;
constexpr int kBtnMaxPayloadForce = 1009;
constexpr int kFieldBase = 1100;
constexpr wchar_t kDefaultVehicleConfigFile[] = L"falcon9_real_defaults.txt";
constexpr wchar_t kEarthTextureFile[] = L"earth_blue_marble.bmp";
constexpr UINT kMsgWorkerDone = WM_APP + 1;
constexpr wchar_t kMainWindowClass[] = L"Falcon9PlannerWinClass";
constexpr COLORREF kUiBg = RGB(240, 240, 240);
constexpr COLORREF kPanelBg = RGB(255, 255, 255);
constexpr COLORREF kPanelBg2 = RGB(232, 236, 240);
constexpr COLORREF kBorderOrange = RGB(170, 170, 170);
constexpr COLORREF kText = RGB(24, 28, 32);
constexpr COLORREF kMutedText = RGB(92, 100, 108);

enum class TaskKind {
    PlanMission,
    PlanBurnoutNoRecovery,
    MaxPayload,
};

enum class CacheTask : uint32_t {
    PlanMission = 1,
    MaxPayload = 2,
};

struct CacheKey {
    CacheTask task = CacheTask::PlanMission;
    MissionRequest request;
};

struct CacheEntry {
    CacheKey key;
    MissionResult result;
};

struct PlannerCache {
    std::filesystem::path path;
    std::vector<CacheEntry> entries;
    bool loaded = false;
    bool available = false;
};

struct CacheWriteRequest {
    bool enabled = false;
    bool forced = false;
    CacheKey key;
};

struct WorkerResult {
    TaskKind task = TaskKind::PlanMission;
    MissionResult plan;
    CacheWriteRequest cache_write;
};

struct Field {
    int id = 0;
    const wchar_t* label = L"";
    const wchar_t* unit = L"";
    double dval = 0.0;
    double MissionRequest::*member = nullptr;
    HWND h_label = nullptr;
    HWND h_edit = nullptr;
    HWND h_unit = nullptr;
};

struct App {
    std::vector<Field> fields;
    HWND btn = nullptr;
    HWND btn_plan_force = nullptr;
    HWND btn_plan_burnout_no_recovery = nullptr;
    HWND btn_max_payload = nullptr;
    HWND btn_max_payload_force = nullptr;
    HWND btn_import_default_cfg = nullptr;
    HWND btn_prev_candidate = nullptr;
    HWND btn_next_candidate = nullptr;
    HWND candidate_label = nullptr;
    HWND list = nullptr;
    HFONT font = nullptr;
    HBRUSH bg_brush = nullptr;
    HBRUSH panel_brush = nullptr;
    RECT plot{0, 0, 0, 0};
    RECT globe_panel{0, 0, 0, 0};
    RECT sweep_panel{0, 0, 0, 0};
    RECT lvd_panel{0, 0, 0, 0};
    RECT initial_state_panel{0, 0, 0, 0};
    RECT final_state_panel{0, 0, 0, 0};
    bool view_initialized = false;
    double view_lat_deg = 20.0;
    double view_lon_deg = -60.0;
    double globe_zoom = 1.0;
    bool dragging_globe = false;
    POINT last_mouse{0, 0};
    MissionRequest base_input;
    bool has_vehicle_config = false;
    std::wstring vehicle_config_path;
    std::wstring vehicle_config_error;
    MissionResult plan;
    PlannerCache cache;
    bool busy = false;
    bool closing = false;
    TaskKind active_task = TaskKind::PlanMission;
    MissionRequest active_request;
    size_t selected_candidate_index = 0;
    std::thread worker;
    std::atomic<bool> cancel_requested{false};
};

struct StartupVehicleConfig {
    bool has_config = false;
    std::wstring config_path;
    std::wstring error;
    MissionRequest input;
};

StartupVehicleConfig g_startup_vehicle;

double clampd(double v, double lo, double hi) {
    return falcon9::clampd(v, lo, hi);
}

double deg2rad(double d) {
    return falcon9::deg2rad(d);
}

double wrap_lon_deg(double lon_deg) {
    return falcon9::wrap_lon_deg(lon_deg);
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

bool parse_utc_timestamp_jd(std::string value, double& jd_out) {
    value = trim_ascii_copy(value);
    if (!value.empty() && (value.back() == 'Z' || value.back() == 'z')) value.pop_back();
    for (char& c : value) {
        if (c == 'T' || c == 't') c = ' ';
    }

    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    char dash1 = 0;
    char dash2 = 0;
    std::istringstream iss(value);
    if (!(iss >> year >> dash1 >> month >> dash2 >> day) || dash1 != '-' || dash2 != '-') return false;

    iss >> std::ws;
    if (!iss.eof()) {
        char colon1 = 0;
        char colon2 = 0;
        if (!(iss >> hour >> colon1 >> minute >> colon2 >> second) || colon1 != ':' || colon2 != ':') return false;
        iss >> std::ws;
        if (!iss.eof()) return false;
    }

    if (
        year < 1900 ||
        month < 1 || month > 12 ||
        day < 1 || day > 31 ||
        hour < 0 || hour > 23 ||
        minute < 0 || minute > 59 ||
        second < 0.0 || second >= 61.0) {
        return false;
    }

    const int a = (14 - month) / 12;
    const int y = year + 4800 - a;
    const int m = month + 12 * a - 3;
    const int jdn = day + (153 * m + 2) / 5 + 365 * y + y / 4 - y / 100 + y / 400 - 32045;
    const double day_fraction = (static_cast<double>(hour) + static_cast<double>(minute) / 60.0 + second / 3600.0) / 24.0;
    jd_out = static_cast<double>(jdn) - 0.5 + day_fraction;
    return std::isfinite(jd_out);
}

std::wstring utc_from_jd(double jd_utc) {
    if (!std::isfinite(jd_utc)) return L"N/A";
    const double unix_s = (jd_utc - 2440587.5) * 86400.0;
    const std::time_t tt = static_cast<std::time_t>(std::llround(unix_s));
    std::tm tm_utc{};
    if (gmtime_s(&tm_utc, &tt) != 0) return L"N/A";
    wchar_t buf[32]{};
    std::wcsftime(buf, std::size(buf), L"%Y-%m-%d %H:%M:%S UTC", &tm_utc);
    return buf;
}

std::wstring fnum(double v, int p) {
    std::wostringstream oss;
    oss << std::fixed << std::setprecision(p) << v;
    return oss.str();
}

bool apply_vehicle_value(MissionRequest& out, const std::string& key_in, double v) {
    const std::string key = lower_ascii_copy(trim_ascii_copy(key_in));
    if (key.empty()) return false;

    if (key == "payload_kg") out.payload_kg = v;
    else if (key == "perigee_km" || key == "orbit_perigee_km") out.perigee_km = v;
    else if (key == "apogee_km" || key == "orbit_apogee_km") out.apogee_km = v;
    else if (key == "cutoff_alt_km" || key == "orbit_cutoff_alt_km") out.cutoff_alt_km = v;
    else if (key == "incl_deg" || key == "inclination_deg" || key == "orbit_inclination_deg") out.incl_deg = v;
    else if (key == "lat_deg" || key == "launch_lat_deg" || key == "launch_latitude_deg") out.lat_deg = v;
    else if (key == "launch_lon_deg" || key == "lon_deg" || key == "launch_longitude_deg") out.launch_lon_deg = v;
    else if (key == "earth_rotation_angle_deg" || key == "body_rotation_angle_deg" || key == "planet_rotation_angle_deg" || key == "rotation_angle_deg") out.earth_rotation_angle_deg = v;
    else if (key == "target_raan_deg" || key == "target_lan_deg" || key == "target_orbit_raan_deg") out.target_raan_deg = v;
    else if (key == "launch_epoch_utc_jd" || key == "launch_utc_jd" || key == "utc_jd") out.launch_epoch_utc_jd = v;
    else if (key == "launch_epoch_utc_mjd" || key == "launch_utc_mjd" || key == "utc_mjd") out.launch_epoch_utc_jd = v + 2400000.5;
    else if (key == "launch_unix_utc_s" || key == "launch_epoch_unix_s" || key == "utc_unix_s") out.launch_epoch_utc_jd = 2440587.5 + v / 86400.0;
    else if (key == "launch_window_half_width_min" || key == "lvd_window_half_width_min") out.launch_window_half_width_min = v;
    else if (key == "ship_downrange_km" || key == "droneship_downrange_km") out.ship_downrange_km = v;
    else if (key == "losses_mps" || key == "ascent_losses_mps") out.losses_mps = v;
    else if (key == "q_limit_kpa" || key == "max_q_limit_kpa") out.q_limit_kpa = v;
    else if (key == "s1_target_maxq_kpa" || key == "stage1_target_maxq_kpa") out.s1_target_maxq_kpa = v;
    else if (key == "s1_target_meco_s" || key == "stage1_target_meco_s") out.s1_target_meco_s = v;
    else if (key == "s1_sep_delay_s" || key == "stage1_sep_delay_s") out.s1_sep_delay_s = v;
    else if (key == "s1_dry_kg" || key == "stage1_dry_kg") out.s1_dry_kg = v;
    else if (key == "s1_prop_kg" || key == "stage1_prop_kg") out.s1_prop_kg = v;
    else if (key == "s1_isp_s" || key == "stage1_isp_s") out.s1_isp_s = v;
    else if (key == "s1_thrust_kn" || key == "stage1_thrust_kn") out.s1_thrust_kN = v;
    else if (key == "s1_reserve" || key == "stage1_reserve" || key == "stage1_reserve_ratio") out.s1_reserve = v;
    else if (key == "s2_dry_kg" || key == "stage2_dry_kg") out.s2_dry_kg = v;
    else if (key == "s2_prop_kg" || key == "stage2_prop_kg") out.s2_prop_kg = v;
    else if (key == "s2_isp_s" || key == "stage2_isp_s") out.s2_isp_s = v;
    else if (key == "s2_thrust_kn" || key == "stage2_thrust_kn") out.s2_thrust_kN = v;
    else if (key == "s2_ignition_delay_s" || key == "stage2_ignition_delay_s") out.s2_ignition_delay_s = v;
    else if (key == "s2_target_seco_s" || key == "stage2_target_seco_s") out.s2_target_seco_s = v;
    else return false;

    return true;
}

bool apply_vehicle_text_value(MissionRequest& out, const std::string& key_in, const std::string& value_in) {
    const std::string key = lower_ascii_copy(trim_ascii_copy(key_in));
    const std::string value = trim_ascii_copy(value_in);
    if (key.empty() || value.empty()) return false;

    if (key == "target_raan_deg" || key == "target_lan_deg" || key == "target_orbit_raan_deg") {
        const std::string lower_value = lower_ascii_copy(value);
        if (lower_value == "auto" || lower_value == "nan" || lower_value == "none") {
            out.target_raan_deg = std::numeric_limits<double>::quiet_NaN();
            return true;
        }
    }

    if (key == "launch_utc" ||
        key == "launch_time_utc" ||
        key == "liftoff_utc" ||
        key == "launch_window_center_utc" ||
        key == "utc") {
        double jd = std::numeric_limits<double>::quiet_NaN();
        if (!parse_utc_timestamp_jd(value, jd)) return false;
        out.launch_epoch_utc_jd = jd;
        return true;
    }

    return false;
}

bool load_vehicle_config(const std::wstring& path, MissionRequest& out, std::wstring& err) {
    std::ifstream in(std::filesystem::path(path), std::ios::in);
    if (!in) {
        err = L"Cannot open vehicle config file.";
        return false;
    }

    MissionRequest cfg{};
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

        if (apply_vehicle_text_value(cfg, key, val)) {
            applied++;
            continue;
        }

        try {
            const double v = std::stod(val);
            if (apply_vehicle_value(cfg, key, v)) applied++;
        } catch (...) {
            continue;
        }
    }

    if (applied <= 0) {
        err = L"Vehicle config has no valid numeric key=value entries.";
        return false;
    }
    out = falcon9::sanitize_request(cfg);
    return true;
}

constexpr char kPlannerCacheMagic[] = {'F', '9', 'P', 'C', 'A', 'C', 'H', 'E'};
constexpr uint32_t kPlannerCacheVersion = 3;
constexpr uint64_t kMaxCacheEntries = 512;
constexpr uint64_t kMaxCacheVectorItems = 1000000;
constexpr uint64_t kMaxCacheTextChars = 1000000;

struct CacheWriter {
    std::ofstream out;
    bool ok = false;

    explicit CacheWriter(const std::filesystem::path& path)
        : out(path, std::ios::binary | std::ios::trunc), ok(static_cast<bool>(out)) {}

    template <typename T>
    void pod(const T& v) {
        if (!ok) return;
        out.write(reinterpret_cast<const char*>(&v), sizeof(T));
        if (!out) ok = false;
    }

    void bytes(const void* data, size_t n) {
        if (!ok || n == 0) return;
        out.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(n));
        if (!out) ok = false;
    }
};

struct CacheReader {
    std::ifstream in;
    bool ok = false;

    explicit CacheReader(const std::filesystem::path& path)
        : in(path, std::ios::binary), ok(static_cast<bool>(in)) {}

    template <typename T>
    bool pod(T& v) {
        if (!ok) return false;
        in.read(reinterpret_cast<char*>(&v), sizeof(T));
        if (!in) {
            ok = false;
            return false;
        }
        return true;
    }

    bool bytes(void* data, size_t n) {
        if (!ok) return false;
        if (n == 0) return true;
        in.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(n));
        if (!in) {
            ok = false;
            return false;
        }
        return true;
    }
};

void write_bool(CacheWriter& w, bool v) {
    const uint8_t b = v ? 1 : 0;
    w.pod(b);
}

bool read_bool(CacheReader& r, bool& v) {
    uint8_t b = 0;
    if (!r.pod(b)) return false;
    v = (b != 0);
    return true;
}

void write_wstring(CacheWriter& w, const std::wstring& s) {
    const uint64_t n = static_cast<uint64_t>(s.size());
    w.pod(n);
    w.bytes(s.data(), static_cast<size_t>(n) * sizeof(wchar_t));
}

bool read_wstring(CacheReader& r, std::wstring& s) {
    uint64_t n = 0;
    if (!r.pod(n) || n > kMaxCacheTextChars) return false;
    s.resize(static_cast<size_t>(n));
    return r.bytes(s.data(), static_cast<size_t>(n) * sizeof(wchar_t));
}

void write_count(CacheWriter& w, size_t n) {
    w.pod(static_cast<uint64_t>(n));
}

bool read_count(CacheReader& r, size_t& n, uint64_t max_items = kMaxCacheVectorItems) {
    uint64_t raw = 0;
    if (!r.pod(raw) || raw > max_items) return false;
    n = static_cast<size_t>(raw);
    return true;
}

bool same_double_key(double a, double b) {
    return a == b || (std::isnan(a) && std::isnan(b));
}

bool same_request_key(const MissionRequest& a, const MissionRequest& b) {
    return
        same_double_key(a.payload_kg, b.payload_kg) &&
        same_double_key(a.perigee_km, b.perigee_km) &&
        same_double_key(a.apogee_km, b.apogee_km) &&
        same_double_key(a.cutoff_alt_km, b.cutoff_alt_km) &&
        same_double_key(a.incl_deg, b.incl_deg) &&
        same_double_key(a.lat_deg, b.lat_deg) &&
        same_double_key(a.launch_lon_deg, b.launch_lon_deg) &&
        same_double_key(a.earth_rotation_angle_deg, b.earth_rotation_angle_deg) &&
        same_double_key(a.target_raan_deg, b.target_raan_deg) &&
        same_double_key(a.launch_epoch_utc_jd, b.launch_epoch_utc_jd) &&
        same_double_key(a.launch_window_half_width_min, b.launch_window_half_width_min) &&
        same_double_key(a.ship_downrange_km, b.ship_downrange_km) &&
        same_double_key(a.losses_mps, b.losses_mps) &&
        same_double_key(a.q_limit_kpa, b.q_limit_kpa) &&
        same_double_key(a.s1_target_maxq_kpa, b.s1_target_maxq_kpa) &&
        same_double_key(a.s1_target_meco_s, b.s1_target_meco_s) &&
        same_double_key(a.s1_sep_delay_s, b.s1_sep_delay_s) &&
        same_double_key(a.s1_dry_kg, b.s1_dry_kg) &&
        same_double_key(a.s1_prop_kg, b.s1_prop_kg) &&
        same_double_key(a.s1_isp_s, b.s1_isp_s) &&
        same_double_key(a.s1_thrust_kN, b.s1_thrust_kN) &&
        same_double_key(a.s1_reserve, b.s1_reserve) &&
        same_double_key(a.s2_dry_kg, b.s2_dry_kg) &&
        same_double_key(a.s2_prop_kg, b.s2_prop_kg) &&
        same_double_key(a.s2_isp_s, b.s2_isp_s) &&
        same_double_key(a.s2_thrust_kN, b.s2_thrust_kN) &&
        same_double_key(a.s2_ignition_delay_s, b.s2_ignition_delay_s) &&
        same_double_key(a.s2_target_seco_s, b.s2_target_seco_s);
}

bool same_cache_key(const CacheKey& a, const CacheKey& b) {
    return a.task == b.task && same_request_key(a.request, b.request);
}

void write_request(CacheWriter& w, const MissionRequest& v) {
    w.pod(v.payload_kg);
    w.pod(v.perigee_km);
    w.pod(v.apogee_km);
    w.pod(v.cutoff_alt_km);
    w.pod(v.incl_deg);
    w.pod(v.lat_deg);
    w.pod(v.launch_lon_deg);
    w.pod(v.earth_rotation_angle_deg);
    w.pod(v.target_raan_deg);
    w.pod(v.launch_epoch_utc_jd);
    w.pod(v.launch_window_half_width_min);
    w.pod(v.ship_downrange_km);
    w.pod(v.losses_mps);
    w.pod(v.q_limit_kpa);
    w.pod(v.s1_target_maxq_kpa);
    w.pod(v.s1_target_meco_s);
    w.pod(v.s1_sep_delay_s);
    w.pod(v.s1_dry_kg);
    w.pod(v.s1_prop_kg);
    w.pod(v.s1_isp_s);
    w.pod(v.s1_thrust_kN);
    w.pod(v.s1_reserve);
    w.pod(v.s2_dry_kg);
    w.pod(v.s2_prop_kg);
    w.pod(v.s2_isp_s);
    w.pod(v.s2_thrust_kN);
    w.pod(v.s2_ignition_delay_s);
    w.pod(v.s2_target_seco_s);
}

bool read_request(CacheReader& r, MissionRequest& v) {
    return
        r.pod(v.payload_kg) &&
        r.pod(v.perigee_km) &&
        r.pod(v.apogee_km) &&
        r.pod(v.cutoff_alt_km) &&
        r.pod(v.incl_deg) &&
        r.pod(v.lat_deg) &&
        r.pod(v.launch_lon_deg) &&
        r.pod(v.earth_rotation_angle_deg) &&
        r.pod(v.target_raan_deg) &&
        r.pod(v.launch_epoch_utc_jd) &&
        r.pod(v.launch_window_half_width_min) &&
        r.pod(v.ship_downrange_km) &&
        r.pod(v.losses_mps) &&
        r.pod(v.q_limit_kpa) &&
        r.pod(v.s1_target_maxq_kpa) &&
        r.pod(v.s1_target_meco_s) &&
        r.pod(v.s1_sep_delay_s) &&
        r.pod(v.s1_dry_kg) &&
        r.pod(v.s1_prop_kg) &&
        r.pod(v.s1_isp_s) &&
        r.pod(v.s1_thrust_kN) &&
        r.pod(v.s1_reserve) &&
        r.pod(v.s2_dry_kg) &&
        r.pod(v.s2_prop_kg) &&
        r.pod(v.s2_isp_s) &&
        r.pod(v.s2_thrust_kN) &&
        r.pod(v.s2_ignition_delay_s) &&
        r.pod(v.s2_target_seco_s);
}

void write_plot_pt(CacheWriter& w, const PlotPt& v) {
    w.pod(v.x_km);
    w.pod(v.y_km);
}

bool read_plot_pt(CacheReader& r, PlotPt& v) {
    return r.pod(v.x_km) && r.pod(v.y_km);
}

void write_sim_pt(CacheWriter& w, const falcon9::SimPt& v) {
    w.pod(v.t);
    w.pod(v.x_km);
    w.pod(v.z_km);
}

bool read_sim_pt(CacheReader& r, falcon9::SimPt& v) {
    return r.pod(v.t) && r.pod(v.x_km) && r.pod(v.z_km);
}

void write_globe_pt(CacheWriter& w, const GlobePt& v) {
    w.pod(v.lat_deg);
    w.pod(v.lon_deg);
    w.pod(v.alt_km);
}

bool read_globe_pt(CacheReader& r, GlobePt& v) {
    return r.pod(v.lat_deg) && r.pod(v.lon_deg) && r.pod(v.alt_km);
}

void write_polar_state(CacheWriter& w, const falcon9::PolarState& v) {
    w.pod(v.r);
    w.pod(v.theta);
    w.pod(v.vr);
    w.pod(v.vt);
    w.pod(v.m);
}

bool read_polar_state(CacheReader& r, falcon9::PolarState& v) {
    return r.pod(v.r) && r.pod(v.theta) && r.pod(v.vr) && r.pod(v.vt) && r.pod(v.m);
}

void write_orbit_metrics(CacheWriter& w, const falcon9::OrbitMetrics& v) {
    w.pod(v.rp_km);
    w.pod(v.ra_km);
    w.pod(v.a_km);
    w.pod(v.e);
    w.pod(v.speed_mps);
    w.pod(v.flight_path_deg);
}

bool read_orbit_metrics(CacheReader& r, falcon9::OrbitMetrics& v) {
    return
        r.pod(v.rp_km) &&
        r.pod(v.ra_km) &&
        r.pod(v.a_km) &&
        r.pod(v.e) &&
        r.pod(v.speed_mps) &&
        r.pod(v.flight_path_deg);
}

void write_orbit_target(CacheWriter& w, const falcon9::OrbitTarget& v) {
    w.pod(v.rp_km);
    w.pod(v.ra_km);
    w.pod(v.cutoff_alt_km);
    w.pod(v.launch_az_deg);
    w.pod(v.r_target_m);
    w.pod(v.vr_target_mps);
    w.pod(v.vt_target_mps);
    w.pod(v.speed_target_mps);
    w.pod(v.fpa_target_deg);
}

bool read_orbit_target(CacheReader& r, falcon9::OrbitTarget& v) {
    return
        r.pod(v.rp_km) &&
        r.pod(v.ra_km) &&
        r.pod(v.cutoff_alt_km) &&
        r.pod(v.launch_az_deg) &&
        r.pod(v.r_target_m) &&
        r.pod(v.vr_target_mps) &&
        r.pod(v.vt_target_mps) &&
        r.pod(v.speed_target_mps) &&
        r.pod(v.fpa_target_deg);
}

void write_sim_pts(CacheWriter& w, const std::vector<falcon9::SimPt>& pts) {
    write_count(w, pts.size());
    for (const auto& pt : pts) write_sim_pt(w, pt);
}

bool read_sim_pts(CacheReader& r, std::vector<falcon9::SimPt>& pts) {
    size_t n = 0;
    if (!read_count(r, n)) return false;
    pts.resize(n);
    for (auto& pt : pts) {
        if (!read_sim_pt(r, pt)) return false;
    }
    return true;
}

void write_stage1_result(CacheWriter& w, const falcon9::Stage1Result& v) {
    write_polar_state(w, v.meco);
    write_polar_state(w, v.sep);
    w.pod(v.burn_s);
    w.pod(v.meco_s);
    w.pod(v.sep_s);
    w.pod(v.guide_start_s);
    w.pod(v.used_prop);
    w.pod(v.rem_prop);
    w.pod(v.max_q);
    w.pod(v.t_max_q);
    w.pod(v.min_throttle);
    w.pod(v.t_min_throttle);
    write_bool(w, v.converged);
    write_bool(w, v.envelope_ok);
    w.pod(v.target_alt_err_km);
    w.pod(v.target_speed_err_mps);
    w.pod(v.target_gamma_err_deg);
    w.pod(v.tgo_final_s);
    w.pod(v.vgo_final_mps);
    write_sim_pts(w, v.traj);
}

bool read_stage1_result(CacheReader& r, falcon9::Stage1Result& v) {
    return
        read_polar_state(r, v.meco) &&
        read_polar_state(r, v.sep) &&
        r.pod(v.burn_s) &&
        r.pod(v.meco_s) &&
        r.pod(v.sep_s) &&
        r.pod(v.guide_start_s) &&
        r.pod(v.used_prop) &&
        r.pod(v.rem_prop) &&
        r.pod(v.max_q) &&
        r.pod(v.t_max_q) &&
        r.pod(v.min_throttle) &&
        r.pod(v.t_min_throttle) &&
        read_bool(r, v.converged) &&
        read_bool(r, v.envelope_ok) &&
        r.pod(v.target_alt_err_km) &&
        r.pod(v.target_speed_err_mps) &&
        r.pod(v.target_gamma_err_deg) &&
        r.pod(v.tgo_final_s) &&
        r.pod(v.vgo_final_mps) &&
        read_sim_pts(r, v.traj);
}

void write_stage2_result(CacheWriter& w, const falcon9::Stage2Result& v) {
    write_polar_state(w, v.ignition);
    write_polar_state(w, v.seco);
    write_orbit_metrics(w, v.orbit);
    w.pod(v.ignition_s);
    w.pod(v.burn_s);
    w.pod(v.cutoff_s);
    w.pod(v.used_prop);
    w.pod(v.rem_prop);
    w.pod(v.target_r_err_km);
    w.pod(v.target_rp_err_km);
    w.pod(v.target_ra_err_km);
    w.pod(v.target_fpa_err_deg);
    w.pod(v.peak_alt_km);
    w.pod(v.orbit_penalty);
    write_bool(w, v.converged);
    write_bool(w, v.orbit_ok);
    w.pod(v.tgo_final_s);
    w.pod(v.vgo_final_mps);
    write_sim_pts(w, v.traj);
}

bool read_stage2_result(CacheReader& r, falcon9::Stage2Result& v) {
    return
        read_polar_state(r, v.ignition) &&
        read_polar_state(r, v.seco) &&
        read_orbit_metrics(r, v.orbit) &&
        r.pod(v.ignition_s) &&
        r.pod(v.burn_s) &&
        r.pod(v.cutoff_s) &&
        r.pod(v.used_prop) &&
        r.pod(v.rem_prop) &&
        r.pod(v.target_r_err_km) &&
        r.pod(v.target_rp_err_km) &&
        r.pod(v.target_ra_err_km) &&
        r.pod(v.target_fpa_err_deg) &&
        r.pod(v.peak_alt_km) &&
        r.pod(v.orbit_penalty) &&
        read_bool(r, v.converged) &&
        read_bool(r, v.orbit_ok) &&
        r.pod(v.tgo_final_s) &&
        r.pod(v.vgo_final_mps) &&
        read_sim_pts(r, v.traj);
}

void write_recovery_result(CacheWriter& w, const falcon9::RecoveryResult& v) {
    write_bool(w, v.feasible);
    write_bool(w, v.converged);
    w.pod(v.landing_ignition_time_s);
    w.pod(v.landing_prop_kg);
    w.pod(v.touchdown_time_s);
    w.pod(v.touchdown_downrange_km);
    w.pod(v.touchdown_lat_deg);
    w.pod(v.touchdown_lon_deg);
    w.pod(v.margin_kg);
    w.pod(v.touchdown_speed_mps);
    write_sim_pts(w, v.coast_traj);
    write_sim_pts(w, v.landing_traj);
}

bool read_recovery_result(CacheReader& r, falcon9::RecoveryResult& v) {
    return
        read_bool(r, v.feasible) &&
        read_bool(r, v.converged) &&
        r.pod(v.landing_ignition_time_s) &&
        r.pod(v.landing_prop_kg) &&
        r.pod(v.touchdown_time_s) &&
        r.pod(v.touchdown_downrange_km) &&
        r.pod(v.touchdown_lat_deg) &&
        r.pod(v.touchdown_lon_deg) &&
        r.pod(v.margin_kg) &&
        r.pod(v.touchdown_speed_mps) &&
        read_sim_pts(r, v.coast_traj) &&
        read_sim_pts(r, v.landing_traj);
}

void write_lvd_event(CacheWriter& w, const falcon9::LvdEvent& v) {
    write_wstring(w, v.name);
    w.pod(v.t_s);
    w.pod(v.alt_km);
    w.pod(v.downrange_km);
    w.pod(v.speed_mps);
    w.pod(v.flight_path_deg);
    w.pod(v.q_kpa);
    w.pod(v.throttle);
    w.pod(v.mass_kg);
}

bool read_lvd_event(CacheReader& r, falcon9::LvdEvent& v) {
    return
        read_wstring(r, v.name) &&
        r.pod(v.t_s) &&
        r.pod(v.alt_km) &&
        r.pod(v.downrange_km) &&
        r.pod(v.speed_mps) &&
        r.pod(v.flight_path_deg) &&
        r.pod(v.q_kpa) &&
        r.pod(v.throttle) &&
        r.pod(v.mass_kg);
}

void write_launch_window_sample(CacheWriter& w, const falcon9::LaunchWindowSample& v) {
    w.pod(v.offset_s);
    w.pod(v.earth_rotation_angle_deg);
    w.pod(v.launch_raan_deg);
    w.pod(v.plane_error_deg);
    w.pod(v.score);
    write_bool(w, v.in_window);
}

bool read_launch_window_sample(CacheReader& r, falcon9::LaunchWindowSample& v) {
    return
        r.pod(v.offset_s) &&
        r.pod(v.earth_rotation_angle_deg) &&
        r.pod(v.launch_raan_deg) &&
        r.pod(v.plane_error_deg) &&
        r.pod(v.score) &&
        read_bool(r, v.in_window);
}

void write_separation_candidate(CacheWriter& w, const SeparationCandidate& v) {
    w.pod(v.sep_time_s);
    w.pod(v.sep_alt_target_km);
    w.pod(v.sep_speed_target_mps);
    w.pod(v.sep_gamma_target_deg);
    write_stage1_result(w, v.stage1);
    write_stage2_result(w, v.stage2);
    write_recovery_result(w, v.recovery);
    write_bool(w, v.feasible);
    w.pod(v.orbit_miss_score);
    w.pod(v.recovery_surplus_kg);
    w.pod(v.score);
}

bool read_separation_candidate(CacheReader& r, SeparationCandidate& v) {
    return
        r.pod(v.sep_time_s) &&
        r.pod(v.sep_alt_target_km) &&
        r.pod(v.sep_speed_target_mps) &&
        r.pod(v.sep_gamma_target_deg) &&
        read_stage1_result(r, v.stage1) &&
        read_stage2_result(r, v.stage2) &&
        read_recovery_result(r, v.recovery) &&
        read_bool(r, v.feasible) &&
        r.pod(v.orbit_miss_score) &&
        r.pod(v.recovery_surplus_kg) &&
        r.pod(v.score);
}

void write_series(CacheWriter& w, const Series& v) {
    write_wstring(w, v.name);
    w.pod(static_cast<uint32_t>(v.color));
    write_count(w, v.pts.size());
    for (const auto& pt : v.pts) write_plot_pt(w, pt);
}

bool read_series(CacheReader& r, Series& v) {
    uint32_t color = 0;
    size_t n = 0;
    if (!read_wstring(r, v.name) || !r.pod(color) || !read_count(r, n)) return false;
    v.color = static_cast<COLORREF>(color);
    v.pts.resize(n);
    for (auto& pt : v.pts) {
        if (!read_plot_pt(r, pt)) return false;
    }
    return true;
}

void write_globe_series(CacheWriter& w, const GlobeSeries& v) {
    write_wstring(w, v.name);
    w.pod(static_cast<uint32_t>(v.color));
    write_count(w, v.pts.size());
    for (const auto& pt : v.pts) write_globe_pt(w, pt);
}

bool read_globe_series(CacheReader& r, GlobeSeries& v) {
    uint32_t color = 0;
    size_t n = 0;
    if (!read_wstring(r, v.name) || !r.pod(color) || !read_count(r, n)) return false;
    v.color = static_cast<COLORREF>(color);
    v.pts.resize(n);
    for (auto& pt : v.pts) {
        if (!read_globe_pt(r, pt)) return false;
    }
    return true;
}

void write_mission_result(CacheWriter& w, const MissionResult& v) {
    write_bool(w, v.ok);
    write_bool(w, v.payload_search_ok);
    write_wstring(w, v.status);
    write_count(w, v.lines.size());
    for (const auto& line : v.lines) write_wstring(w, line);
    write_count(w, v.profile_series.size());
    for (const auto& series : v.profile_series) write_series(w, series);
    write_count(w, v.separation_time_series.size());
    for (const auto& series : v.separation_time_series) write_series(w, series);
    write_count(w, v.lvd_time_series.size());
    for (const auto& series : v.lvd_time_series) write_series(w, series);
    write_count(w, v.separation_candidates.size());
    for (const auto& cand : v.separation_candidates) write_separation_candidate(w, cand);
    write_count(w, v.lvd_events.size());
    for (const auto& ev : v.lvd_events) write_lvd_event(w, ev);
    write_count(w, v.launch_window_samples.size());
    for (const auto& s : v.launch_window_samples) write_launch_window_sample(w, s);
    write_count(w, v.globe_series.size());
    for (const auto& series : v.globe_series) write_globe_series(w, series);
    w.pod(v.launch_lat_deg);
    w.pod(v.launch_lon_deg);
    w.pod(v.launch_epoch_utc_jd);
    w.pod(v.lvd_launch_offset_s);
    w.pod(v.lvd_earth_rotation_angle_deg);
    w.pod(v.lvd_launch_raan_deg);
    w.pod(v.lvd_target_raan_deg);
    w.pod(v.lvd_plane_error_deg);
    w.pod(v.ship_lat_deg);
    w.pod(v.ship_lon_deg);
    w.pod(v.view_lat_deg);
    w.pod(v.view_lon_deg);
    write_orbit_target(w, v.orbit_target);
    write_separation_candidate(w, v.best_candidate);
    write_stage1_result(w, v.stage1);
    write_stage2_result(w, v.stage2);
    write_recovery_result(w, v.recovery);
}

bool read_mission_result(CacheReader& r, MissionResult& v) {
    size_t n = 0;
    if (!read_bool(r, v.ok) ||
        !read_bool(r, v.payload_search_ok) ||
        !read_wstring(r, v.status) ||
        !read_count(r, n)) {
        return false;
    }
    v.lines.resize(n);
    for (auto& line : v.lines) {
        if (!read_wstring(r, line)) return false;
    }

    if (!read_count(r, n)) return false;
    v.profile_series.resize(n);
    for (auto& series : v.profile_series) {
        if (!read_series(r, series)) return false;
    }

    if (!read_count(r, n)) return false;
    v.separation_time_series.resize(n);
    for (auto& series : v.separation_time_series) {
        if (!read_series(r, series)) return false;
    }

    if (!read_count(r, n)) return false;
    v.lvd_time_series.resize(n);
    for (auto& series : v.lvd_time_series) {
        if (!read_series(r, series)) return false;
    }

    if (!read_count(r, n)) return false;
    v.separation_candidates.resize(n);
    for (auto& cand : v.separation_candidates) {
        if (!read_separation_candidate(r, cand)) return false;
    }

    if (!read_count(r, n)) return false;
    v.lvd_events.resize(n);
    for (auto& ev : v.lvd_events) {
        if (!read_lvd_event(r, ev)) return false;
    }

    if (!read_count(r, n)) return false;
    v.launch_window_samples.resize(n);
    for (auto& s : v.launch_window_samples) {
        if (!read_launch_window_sample(r, s)) return false;
    }

    if (!read_count(r, n)) return false;
    v.globe_series.resize(n);
    for (auto& series : v.globe_series) {
        if (!read_globe_series(r, series)) return false;
    }

    return
        r.pod(v.launch_lat_deg) &&
        r.pod(v.launch_lon_deg) &&
        r.pod(v.launch_epoch_utc_jd) &&
        r.pod(v.lvd_launch_offset_s) &&
        r.pod(v.lvd_earth_rotation_angle_deg) &&
        r.pod(v.lvd_launch_raan_deg) &&
        r.pod(v.lvd_target_raan_deg) &&
        r.pod(v.lvd_plane_error_deg) &&
        r.pod(v.ship_lat_deg) &&
        r.pod(v.ship_lon_deg) &&
        r.pod(v.view_lat_deg) &&
        r.pod(v.view_lon_deg) &&
        read_orbit_target(r, v.orbit_target) &&
        read_separation_candidate(r, v.best_candidate) &&
        read_stage1_result(r, v.stage1) &&
        read_stage2_result(r, v.stage2) &&
        read_recovery_result(r, v.recovery);
}

void write_cache_key(CacheWriter& w, const CacheKey& key) {
    w.pod(static_cast<uint32_t>(key.task));
    write_request(w, key.request);
}

bool read_cache_key(CacheReader& r, CacheKey& key) {
    uint32_t raw_task = 0;
    if (!r.pod(raw_task) || !read_request(r, key.request)) return false;
    if (raw_task == static_cast<uint32_t>(CacheTask::PlanMission)) {
        key.task = CacheTask::PlanMission;
    } else if (raw_task == static_cast<uint32_t>(CacheTask::MaxPayload)) {
        key.task = CacheTask::MaxPayload;
    } else {
        return false;
    }
    key.request = falcon9::sanitize_request(key.request);
    return true;
}

std::filesystem::path planner_cache_file_path() {
    wchar_t local_appdata[32768]{};
    const DWORD n = GetEnvironmentVariableW(L"LOCALAPPDATA", local_appdata, static_cast<DWORD>(std::size(local_appdata)));
    if (n == 0 || n >= std::size(local_appdata)) return {};
    return std::filesystem::path(local_appdata) / L"Falcon9Planner" / L"planner_cache_v1.bin";
}

bool load_planner_cache(PlannerCache& cache) {
    cache.path = planner_cache_file_path();
    cache.entries.clear();
    cache.loaded = true;
    cache.available = !cache.path.empty();
    if (!cache.available) return false;

    std::error_code ec;
    if (!std::filesystem::exists(cache.path, ec)) return true;

    CacheReader r(cache.path);
    char magic[sizeof(kPlannerCacheMagic)]{};
    uint32_t version = 0;
    size_t count = 0;
    if (!r.bytes(magic, sizeof(magic)) ||
        std::memcmp(magic, kPlannerCacheMagic, sizeof(magic)) != 0 ||
        !r.pod(version) ||
        version != kPlannerCacheVersion ||
        !read_count(r, count, kMaxCacheEntries)) {
        cache.entries.clear();
        return false;
    }

    cache.entries.resize(count);
    for (auto& entry : cache.entries) {
        if (!read_cache_key(r, entry.key) || !read_mission_result(r, entry.result)) {
            cache.entries.clear();
            return false;
        }
    }
    return true;
}

bool save_planner_cache(const PlannerCache& cache) {
    if (!cache.available || cache.path.empty()) return false;

    std::error_code ec;
    std::filesystem::create_directories(cache.path.parent_path(), ec);
    if (ec) return false;

    std::filesystem::path tmp = cache.path;
    tmp += L".tmp";

    {
        CacheWriter w(tmp);
        w.bytes(kPlannerCacheMagic, sizeof(kPlannerCacheMagic));
        w.pod(kPlannerCacheVersion);
        write_count(w, cache.entries.size());
        for (const auto& entry : cache.entries) {
            write_cache_key(w, entry.key);
            write_mission_result(w, entry.result);
        }
        if (!w.ok) {
            DeleteFileW(tmp.c_str());
            return false;
        }
        w.out.close();
        if (!w.out) {
            DeleteFileW(tmp.c_str());
            return false;
        }
    }

    if (!MoveFileExW(tmp.c_str(), cache.path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
        DeleteFileW(tmp.c_str());
        return false;
    }
    return true;
}

bool find_cached_result(const PlannerCache& cache, const CacheKey& key, MissionResult& out) {
    for (const auto& entry : cache.entries) {
        if (same_cache_key(entry.key, key)) {
            out = entry.result;
            return true;
        }
    }
    return false;
}

bool store_cached_result(PlannerCache& cache, const CacheKey& key, const MissionResult& result) {
    if (!cache.loaded) load_planner_cache(cache);
    if (!cache.available) return false;

    for (auto& entry : cache.entries) {
        if (same_cache_key(entry.key, key)) {
            entry.result = result;
            return save_planner_cache(cache);
        }
    }

    if (cache.entries.size() >= kMaxCacheEntries) {
        cache.entries.erase(cache.entries.begin());
    }
    cache.entries.push_back({key, result});
    return save_planner_cache(cache);
}

CacheKey make_cache_key(CacheTask task, const MissionRequest& request) {
    CacheKey key;
    key.task = task;
    key.request = falcon9::sanitize_request(request);
    return key;
}

std::filesystem::path planner_module_dir() {
    wchar_t module_buf[MAX_PATH]{};
    const DWORD n = GetModuleFileNameW(nullptr, module_buf, MAX_PATH);
    if (n == 0 || n >= MAX_PATH) return {};
    return std::filesystem::path(module_buf).parent_path();
}

bool is_regular_file_path(const std::filesystem::path& p) {
    std::error_code ec;
    return !p.empty() && std::filesystem::exists(p, ec) && std::filesystem::is_regular_file(p, ec);
}

bool find_default_vehicle_config_path(std::wstring& out_path) {
    std::vector<std::filesystem::path> candidates;
    const std::filesystem::path exe_dir = planner_module_dir();
    if (!exe_dir.empty()) {
        candidates.push_back(exe_dir / kDefaultVehicleConfigFile);
        candidates.push_back(exe_dir / L"config" / kDefaultVehicleConfigFile);
        const std::filesystem::path p1 = exe_dir.parent_path();
        if (!p1.empty()) {
            candidates.push_back(p1 / kDefaultVehicleConfigFile);
            candidates.push_back(p1 / L"config" / kDefaultVehicleConfigFile);
            const std::filesystem::path p2 = p1.parent_path();
            if (!p2.empty()) {
                candidates.push_back(p2 / kDefaultVehicleConfigFile);
                candidates.push_back(p2 / L"config" / kDefaultVehicleConfigFile);
            }
        }
    }
    candidates.push_back(std::filesystem::current_path() / kDefaultVehicleConfigFile);
    candidates.push_back(std::filesystem::current_path() / L"config" / kDefaultVehicleConfigFile);

    for (const auto& p : candidates) {
        if (is_regular_file_path(p)) {
            out_path = p.wstring();
            return true;
        }
    }
    return false;
}

void sync_fields_from_base_input(App& a) {
    MissionRequest s = falcon9::sanitize_request(a.base_input);
    for (Field& f : a.fields) {
        f.dval = s.*(f.member);
        if (f.member == &MissionRequest::cutoff_alt_km && !std::isfinite(f.dval)) f.dval = s.perigee_km;
        const std::wstring txt = fnum(f.dval, std::abs(f.dval) < 1.0 ? 3 : 1);
        SetWindowTextW(f.h_edit, txt.c_str());
    }
}

bool load_vehicle_config_into_app(App& a, const std::wstring& path, std::wstring& err) {
    MissionRequest loaded{};
    if (!load_vehicle_config(path, loaded, err)) return false;
    a.base_input = loaded;
    a.has_vehicle_config = true;
    a.vehicle_config_path = path;
    a.vehicle_config_error.clear();
    sync_fields_from_base_input(a);
    return true;
}

std::vector<Field> make_fields(const MissionRequest& seed) {
    MissionRequest d = falcon9::sanitize_request(seed);
    int id = kFieldBase;
    return {
        {id++, L"Perigee Altitude", L"km", d.perigee_km, &MissionRequest::perigee_km},
        {id++, L"Apogee Altitude", L"km", d.apogee_km, &MissionRequest::apogee_km},
        {id++, L"Cutoff Altitude", L"km", std::isfinite(d.cutoff_alt_km) ? d.cutoff_alt_km : d.perigee_km, &MissionRequest::cutoff_alt_km},
        {id++, L"Inclination", L"deg", d.incl_deg, &MissionRequest::incl_deg},
    };
}

App* app(HWND hwnd) {
    return reinterpret_cast<App*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
}

void set_font(HWND h, HFONT f) {
    if (h && f) SendMessageW(h, WM_SETFONT, reinterpret_cast<WPARAM>(f), TRUE);
}

void split_plot_rects(const RECT& outer, RECT& left, RECT& right) {
    const int split = outer.left + static_cast<int>((outer.right - outer.left) * 0.50);
    left = outer;
    right = outer;
    left.right = split - 4;
    right.left = split + 4;
}

void split_plot_rects_three(const RECT& outer, RECT& left, RECT& middle, RECT& right) {
    const int w = outer.right - outer.left;
    const int split1 = outer.left + w / 3;
    const int split2 = outer.left + (2 * w) / 3;
    left = outer;
    middle = outer;
    right = outer;
    left.right = split1 - 4;
    middle.left = split1 + 4;
    middle.right = split2 - 4;
    right.left = split2 + 4;
}

void layout(HWND hwnd, App& a) {
    RECT rc{};
    GetClientRect(hwnd, &rc);
    const int left_w = 520;
    const int x0 = 12;
    int y = 12;
    const int row_h = 22;
    const int gap = 4;
    const int lw = 230;
    const int ew = 140;
    const int uw = 64;

    for (Field& f : a.fields) {
        MoveWindow(f.h_label, x0, y + 2, lw, row_h, TRUE);
        MoveWindow(f.h_edit, x0 + lw + 8, y, ew, row_h, TRUE);
        MoveWindow(f.h_unit, x0 + lw + ew + 14, y + 2, uw, row_h, TRUE);
        y += row_h + gap;
    }
    const int button_w = left_w - 24;
    const int force_w = 140;
    const int split_gap = 8;
    const int main_w = button_w - force_w - split_gap;
    MoveWindow(a.btn, x0, y + 8, main_w, 30, TRUE);
    MoveWindow(a.btn_plan_force, x0 + main_w + split_gap, y + 8, force_w, 30, TRUE);
    MoveWindow(a.btn_plan_burnout_no_recovery, x0, y + 44, left_w - 24, 30, TRUE);
    MoveWindow(a.btn_max_payload, x0, y + 80, main_w, 30, TRUE);
    MoveWindow(a.btn_max_payload_force, x0 + main_w + split_gap, y + 80, force_w, 30, TRUE);
    MoveWindow(a.btn_import_default_cfg, x0, y + 116, left_w - 24, 30, TRUE);
    MoveWindow(a.btn_prev_candidate, x0, y + 156, 48, 28, TRUE);
    MoveWindow(a.btn_next_candidate, x0 + 56, y + 156, 48, 28, TRUE);
    MoveWindow(a.candidate_label, x0 + 116, y + 160, left_w - 140, 24, TRUE);

    const int pl = left_w + 16;
    const int pt = 12;
    const int pr = std::max(pl + 240, static_cast<int>(rc.right) - 14);
    const int rb = std::max(pt + 520, static_cast<int>(rc.bottom) - 12);

    a.plot = {pl, pt, pr, rb};
    a.globe_panel = a.plot;
    a.sweep_panel = {0, 0, 0, 0};
    a.lvd_panel = {0, 0, 0, 0};

    const int state_top = y + 198;
    const int state_h = std::max(118, std::min(152, (static_cast<int>(rc.bottom) - state_top - 180) / 2));
    a.initial_state_panel = {x0, state_top, left_w - 12, state_top + state_h};
    a.final_state_panel = {x0, a.initial_state_panel.bottom + 8, left_w - 12, a.initial_state_panel.bottom + 8 + state_h};

    const int list_top = a.final_state_panel.bottom + 8;
    MoveWindow(
        a.list,
        x0,
        list_top,
        left_w - 24,
        std::max(80, static_cast<int>(rc.bottom) - list_top - 12),
        TRUE);
}

bool read_inputs(const App& a, MissionRequest& in, std::wstring& err) {
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
    in = falcon9::sanitize_request(in);
    return true;
}

void refresh_list(const App& a) {
    SendMessageW(a.list, LB_RESETCONTENT, 0, 0);
    for (const std::wstring& s : a.plan.lines) {
        SendMessageW(a.list, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(s.c_str()));
    }
}

const SeparationCandidate* selected_candidate(const App& a) {
    if (a.selected_candidate_index >= a.plan.separation_candidates.size()) return nullptr;
    return &a.plan.separation_candidates[a.selected_candidate_index];
}

void update_candidate_label(App& a) {
    std::wstring text = L"Candidates: none";
    if (const SeparationCandidate* cand = selected_candidate(a)) {
        text =
            std::to_wstring(a.selected_candidate_index + 1) + L"/" +
            std::to_wstring(a.plan.separation_candidates.size()) +
            L" sep=" + fnum(cand->sep_time_s, 1) +
            L" s, S2=" + fnum(cand->stage2.rem_prop, 1) + L" kg";
    }
    if (a.candidate_label) SetWindowTextW(a.candidate_label, text.c_str());
}

void sync_candidate_controls(App& a) {
    if (a.selected_candidate_index >= a.plan.separation_candidates.size()) {
        a.selected_candidate_index = a.plan.separation_candidates.empty() ? 0 : a.plan.separation_candidates.size() - 1;
    }
    update_candidate_label(a);
    EnableWindow(a.btn_prev_candidate, (!a.busy && a.selected_candidate_index > 0) ? TRUE : FALSE);
    EnableWindow(
        a.btn_next_candidate,
        (!a.busy && a.selected_candidate_index + 1 < a.plan.separation_candidates.size()) ? TRUE : FALSE);
}

void select_best_candidate_index(App& a) {
    a.selected_candidate_index = 0;
    if (a.plan.separation_candidates.empty() || !std::isfinite(a.plan.best_candidate.sep_time_s)) return;

    double best_dt = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < a.plan.separation_candidates.size(); ++i) {
        const double dt = std::abs(a.plan.separation_candidates[i].sep_time_s - a.plan.best_candidate.sep_time_s);
        if (dt < best_dt) {
            best_dt = dt;
            a.selected_candidate_index = i;
        }
    }
}

void prepend_vehicle_config_note(App& a) {
    size_t insert_pos = 0;
    if (a.has_vehicle_config) {
        a.plan.lines.insert(a.plan.lines.begin(), L"[Vehicle Config] Loaded from: " + a.vehicle_config_path);
        insert_pos = 1;
    } else if (!a.vehicle_config_error.empty()) {
        a.plan.lines.insert(a.plan.lines.begin(), L"[Vehicle Config] Load failed: " + a.vehicle_config_error);
        insert_pos = 1;
    } else {
        a.plan.lines.insert(a.plan.lines.begin(), L"[Vehicle Config] Not provided; using internal defaults.");
        insert_pos = 1;
    }

    const MissionRequest in = falcon9::sanitize_request(a.active_request);
    a.plan.lines.insert(
        a.plan.lines.begin() + static_cast<std::ptrdiff_t>(insert_pos++),
        L"[Vehicle Input] payload=" + fnum(in.payload_kg, 1) +
            L" kg, S1 dry=" + fnum(in.s1_dry_kg, 1) +
            L" kg, prop=" + fnum(in.s1_prop_kg, 1) +
            L" kg, thrust=" + fnum(in.s1_thrust_kN, 1) +
            L" kN, Isp=" + fnum(in.s1_isp_s, 1) + L" s");
    a.plan.lines.insert(
        a.plan.lines.begin() + static_cast<std::ptrdiff_t>(insert_pos++),
        L"[Vehicle Input] S2 dry=" + fnum(in.s2_dry_kg, 1) +
            L" kg, prop=" + fnum(in.s2_prop_kg, 1) +
            L" kg, thrust=" + fnum(in.s2_thrust_kN, 1) +
            L" kN, Isp=" + fnum(in.s2_isp_s, 1) +
            L" s, q_limit=" + fnum(in.q_limit_kpa, 1) + L" kPa");
    a.plan.lines.insert(
        a.plan.lines.begin() + static_cast<std::ptrdiff_t>(insert_pos++),
        L"[LVD Input] earth_rotation=" + fnum(in.earth_rotation_angle_deg, 3) +
            L" deg, target_RAAN=" +
            (std::isfinite(in.target_raan_deg) ? fnum(in.target_raan_deg, 3) : std::wstring(L"auto")) +
            L", window_half_width=" + fnum(in.launch_window_half_width_min, 1) + L" min");
    if (std::isfinite(in.launch_epoch_utc_jd)) {
        a.plan.lines.insert(
            a.plan.lines.begin() + static_cast<std::ptrdiff_t>(insert_pos++),
            L"[UTC Input] center=" + utc_from_jd(in.launch_epoch_utc_jd) +
                L", selected_liftoff=" + utc_from_jd(in.launch_epoch_utc_jd + a.plan.lvd_launch_offset_s / 86400.0) +
                L", selected_offset=" + fnum(a.plan.lvd_launch_offset_s / 60.0, 2) + L" min");
    }
}

void join_worker_if_needed(App& a) {
    if (a.worker.joinable()) a.worker.join();
}

void discard_worker_messages(HWND hwnd) {
    MSG msg{};
    while (PeekMessageW(&msg, hwnd, kMsgWorkerDone, kMsgWorkerDone, PM_REMOVE)) {
        delete reinterpret_cast<WorkerResult*>(msg.lParam);
    }
}

void set_controls_enabled(const App& a, bool enabled) {
    for (const Field& f : a.fields) {
        EnableWindow(f.h_edit, enabled ? TRUE : FALSE);
    }
    EnableWindow(a.btn, enabled ? TRUE : FALSE);
    EnableWindow(a.btn_plan_force, enabled ? TRUE : FALSE);
    EnableWindow(a.btn_plan_burnout_no_recovery, enabled ? TRUE : FALSE);
    EnableWindow(a.btn_max_payload, enabled ? TRUE : FALSE);
    EnableWindow(a.btn_max_payload_force, enabled ? TRUE : FALSE);
    EnableWindow(a.btn_import_default_cfg, enabled ? TRUE : FALSE);
    const bool have_candidates = enabled && !a.plan.separation_candidates.empty();
    EnableWindow(a.btn_prev_candidate, have_candidates && a.selected_candidate_index > 0 ? TRUE : FALSE);
    EnableWindow(
        a.btn_next_candidate,
        have_candidates && a.selected_candidate_index + 1 < a.plan.separation_candidates.size() ? TRUE : FALSE);
}

void show_busy_placeholder(App& a, TaskKind task) {
    a.plan = MissionResult{};
    if (task == TaskKind::MaxPayload) {
        a.plan.lines.push_back(L"[Payload Capability] Searching max payload...");
        a.plan.lines.push_back(L"[Payload Capability] Search starts from payload = 0 kg.");
    } else if (task == TaskKind::PlanBurnoutNoRecovery) {
        a.plan.lines.push_back(L"[Status] Computing no-recovery burnout separation plan...");
    } else {
        a.plan.lines.push_back(L"[Status] Computing...");
    }
}

MissionResult solve_plan_request(const MissionRequest& in, std::atomic<bool>* cancel_requested) {
    return falcon9::solve_mission(in, {cancel_requested, 0});
}

MissionResult solve_plan_burnout_no_recovery_request(const MissionRequest& in, std::atomic<bool>* cancel_requested) {
    falcon9::SolveControl control;
    control.cancel_requested = cancel_requested;
    control.ignore_recovery = true;
    control.force_stage1_burnout = true;
    return falcon9::solve_mission(in, control);
}

bool payload_margin_search_ok(const MissionResult& result) {
    return falcon9::mission_payload_search_ok(result) &&
           result.recovery.feasible &&
           result.recovery.margin_kg >= -1e-3;
}

MissionResult solve_max_payload_request(const MissionRequest& in, std::atomic<bool>* cancel_requested) {
    constexpr double kPayloadSearchCapKg = 30000.0;
    const double coarse_steps[] = {4000.0, 1000.0, 200.0, 50.0};
    constexpr double kFinalStepKg = 10.0;
    const double payload_cap_kg = std::max(kPayloadSearchCapKg, in.payload_kg);
    bool hit_search_cap = false;
    unsigned max_parallel_workers_used = 1;
    size_t parallel_probe_count = 0;

    auto eval_payload = [&](double payload_kg, unsigned mission_worker_count) {
        MissionRequest test = in;
        test.payload_kg = std::max(0.0, payload_kg);
        falcon9::SolveControl control;
        control.cancel_requested = cancel_requested;
        control.worker_count = mission_worker_count;
        return falcon9::solve_mission(test, control);
    };

    struct PayloadProbe {
        double payload_kg = 0.0;
        MissionResult plan;
        bool completed = false;
    };

    auto resolve_outer_workers = [](size_t probe_count) {
        unsigned workers = std::thread::hardware_concurrency();
        if (workers == 0) workers = 1;
        workers = std::max(1u, std::min<unsigned>(workers, 6u));
        workers = std::min<unsigned>(workers, static_cast<unsigned>(std::max<size_t>(1, probe_count)));
        return workers;
    };

    auto run_parallel_payload_batch = [&](const std::vector<double>& payloads) {
        std::vector<PayloadProbe> probes(payloads.size());
        if (payloads.empty()) return probes;

        const unsigned workers = resolve_outer_workers(payloads.size());
        max_parallel_workers_used = std::max(max_parallel_workers_used, workers);
        parallel_probe_count += payloads.size();
        std::atomic<size_t> next_index{0};

        auto worker_fn = [&]() {
            while (!cancel_requested->load(std::memory_order_relaxed)) {
                const size_t i = next_index.fetch_add(1, std::memory_order_relaxed);
                if (i >= payloads.size()) break;
                probes[i].payload_kg = payloads[i];
                probes[i].plan = eval_payload(payloads[i], 1);
                probes[i].completed = !cancel_requested->load(std::memory_order_relaxed);
            }
        };

        std::vector<std::thread> threads;
        threads.reserve(workers > 1 ? workers - 1 : 0);
        for (unsigned worker_index = 1; worker_index < workers; ++worker_index) {
            threads.emplace_back(worker_fn);
        }
        worker_fn();
        for (std::thread& thread : threads) {
            thread.join();
        }
        return probes;
    };

    MissionResult plan_zero = eval_payload(0.0, 0);
    if (!falcon9::mission_payload_search_ok(plan_zero)) {
        plan_zero.lines.insert(plan_zero.lines.begin(), L"[Payload Capability] Max payload = N/A");
        plan_zero.lines.insert(plan_zero.lines.begin() + 1, L"[Payload Capability] No orbit-and-landing-feasible candidate at payload = 0 kg.");
        return plan_zero;
    }

    bool have_positive_margin = payload_margin_search_ok(plan_zero);
    double best_payload_kg = 0.0;
    MissionResult best_plan = plan_zero;
    double first_fail_payload_kg = std::numeric_limits<double>::quiet_NaN();

    for (double step_kg : coarse_steps) {
        if (cancel_requested->load(std::memory_order_relaxed)) break;
        const double upper_payload_kg = std::isfinite(first_fail_payload_kg)
            ? std::min(first_fail_payload_kg, payload_cap_kg)
            : payload_cap_kg;
        std::vector<double> payloads;
        for (double probe_payload_kg = best_payload_kg + step_kg;
             probe_payload_kg <= upper_payload_kg + 1e-6;
             probe_payload_kg += step_kg) {
            payloads.push_back(std::min(probe_payload_kg, payload_cap_kg));
            if (probe_payload_kg >= payload_cap_kg - 1e-6) break;
        }
        payloads.erase(std::unique(payloads.begin(), payloads.end(), [](double a, double b) {
            return std::abs(a - b) <= 1e-6;
        }), payloads.end());
        if (payloads.empty()) continue;

        std::vector<PayloadProbe> probes = run_parallel_payload_batch(payloads);
        int consecutive_failures = 0;
        double local_first_fail_payload_kg = std::numeric_limits<double>::quiet_NaN();
        for (PayloadProbe& probe : probes) {
            if (cancel_requested->load(std::memory_order_relaxed)) break;
            if (!probe.completed) continue;
            const double probe_payload_kg = probe.payload_kg;
            const bool capped_probe = probe_payload_kg >= payload_cap_kg - 1e-6;
            if (!payload_margin_search_ok(probe.plan)) {
                if (!std::isfinite(local_first_fail_payload_kg)) local_first_fail_payload_kg = probe_payload_kg;
                ++consecutive_failures;
                if (consecutive_failures >= 3 || capped_probe) {
                    first_fail_payload_kg = local_first_fail_payload_kg;
                    break;
                }
                continue;
            }

            have_positive_margin = true;
            best_payload_kg = probe_payload_kg;
            best_plan = std::move(probe.plan);
            consecutive_failures = 0;
            local_first_fail_payload_kg = std::numeric_limits<double>::quiet_NaN();
            if (capped_probe) {
                hit_search_cap = true;
                break;
            }
        }
        if (hit_search_cap) break;
    }

    int consecutive_final_failures = 0;
    double local_first_final_fail_payload_kg = std::numeric_limits<double>::quiet_NaN();
    while (!cancel_requested->load(std::memory_order_relaxed)) {
        const double raw_probe_payload_kg = best_payload_kg + kFinalStepKg * static_cast<double>(consecutive_final_failures + 1);
        const double probe_payload_kg = std::min(raw_probe_payload_kg, payload_cap_kg);
        const bool capped_probe = raw_probe_payload_kg > payload_cap_kg + 1e-6;
        MissionResult probe = eval_payload(probe_payload_kg, 0);
        if (!payload_margin_search_ok(probe)) {
            if (!std::isfinite(local_first_final_fail_payload_kg)) local_first_final_fail_payload_kg = probe_payload_kg;
            ++consecutive_final_failures;
            if (consecutive_final_failures >= 3 || capped_probe) {
                first_fail_payload_kg = local_first_final_fail_payload_kg;
                break;
            }
            continue;
        }
        have_positive_margin = true;
        best_payload_kg = probe_payload_kg;
        best_plan = std::move(probe);
        consecutive_final_failures = 0;
        local_first_final_fail_payload_kg = std::numeric_limits<double>::quiet_NaN();
        if (capped_probe) {
            hit_search_cap = true;
            break;
        }
    }

    if (!have_positive_margin && plan_zero.recovery.margin_kg <= 0.0) {
        best_plan = plan_zero;
        best_payload_kg = 0.0;
        if (!std::isfinite(first_fail_payload_kg)) first_fail_payload_kg = kFinalStepKg;
    } else if (!std::isfinite(first_fail_payload_kg)) {
        first_fail_payload_kg = hit_search_cap ? payload_cap_kg : best_payload_kg + kFinalStepKg;
    }

    best_plan.lines.insert(best_plan.lines.begin(), L"[Payload Capability] Max payload at current orbit ~ " + fnum(best_payload_kg, 1) + L" kg");
    best_plan.lines.insert(best_plan.lines.begin() + 1, L"[Payload Capability] Parallel coarse probes = " + std::to_wstring(parallel_probe_count) + L", workers = " + std::to_wstring(max_parallel_workers_used));
    best_plan.lines.insert(best_plan.lines.begin() + 2, L"[Payload Capability] Search start payload = 0.0 kg");
    best_plan.lines.insert(best_plan.lines.begin() + 3, L"[Payload Capability] Landing margin at max payload = " + fnum(best_plan.recovery.margin_kg, 1) + L" kg");
    best_plan.lines.insert(best_plan.lines.begin() + 4, L"[Payload Capability] Feasible interval: [" + fnum(best_payload_kg, 1) + L", " + fnum(first_fail_payload_kg, 1) + L"] kg");
    if (hit_search_cap) {
        best_plan.lines.insert(best_plan.lines.begin() + 5, L"[Payload Capability] Search cap reached before landing margin crossed zero.");
    }
    return best_plan;
}

void launch_task(HWND hwnd, App& a, TaskKind task, const MissionRequest& in, CacheWriteRequest cache_write = {}) {
    if (a.busy) return;

    join_worker_if_needed(a);
    a.busy = true;
    a.active_task = task;
    a.active_request = in;
    a.cancel_requested.store(false, std::memory_order_relaxed);
    set_controls_enabled(a, false);
    show_busy_placeholder(a, task);
    sync_candidate_controls(a);
    refresh_list(a);
    InvalidateRect(hwnd, nullptr, TRUE);

    try {
        a.worker = std::thread([hwnd, task, in, cache_write, cancel_requested = &a.cancel_requested]() {
            std::unique_ptr<WorkerResult> result = std::make_unique<WorkerResult>();
            result->task = task;
            result->cache_write = cache_write;
            if (task == TaskKind::MaxPayload) {
                result->plan = solve_max_payload_request(in, cancel_requested);
            } else if (task == TaskKind::PlanBurnoutNoRecovery) {
                result->plan = solve_plan_burnout_no_recovery_request(in, cancel_requested);
            } else {
                result->plan = solve_plan_request(in, cancel_requested);
            }
            if (cancel_requested->load(std::memory_order_relaxed)) return;
            WorkerResult* raw = result.release();
            if (!PostMessageW(hwnd, kMsgWorkerDone, 0, reinterpret_cast<LPARAM>(raw))) {
                delete raw;
            }
        });
    } catch (...) {
        a.busy = false;
        a.cancel_requested.store(false, std::memory_order_relaxed);
        set_controls_enabled(a, true);
        a.plan = MissionResult{};
        a.plan.lines.push_back(L"[Status] Failed to start background worker.");
        refresh_list(a);
        InvalidateRect(hwnd, nullptr, TRUE);
    }
}

void display_completed_plan(
    HWND hwnd,
    App& a,
    TaskKind task,
    const MissionRequest& in,
    MissionResult plan,
    const std::wstring& cache_note = L"") {
    a.active_task = task;
    a.active_request = in;
    a.plan = std::move(plan);
    prepend_vehicle_config_note(a);
    if (!cache_note.empty()) {
        a.plan.lines.insert(a.plan.lines.begin(), cache_note);
    }
    select_best_candidate_index(a);
    set_controls_enabled(a, true);
    sync_candidate_controls(a);
    if (!a.view_initialized) {
        a.view_lat_deg = a.plan.view_lat_deg;
        a.view_lon_deg = a.plan.view_lon_deg;
        a.view_initialized = true;
    }
    refresh_list(a);
    InvalidateRect(hwnd, nullptr, TRUE);
}

void finish_task(HWND hwnd, App& a, std::unique_ptr<WorkerResult> result) {
    join_worker_if_needed(a);
    a.busy = false;
    a.cancel_requested.store(false, std::memory_order_relaxed);

    std::wstring cache_note;
    if (result->cache_write.enabled) {
        const bool saved = store_cached_result(a.cache, result->cache_write.key, result->plan);
        if (result->cache_write.forced) {
            cache_note = saved
                ? L"[Cache] Recomputed and updated cache."
                : L"[Cache] Recomputed; cache update failed.";
        }
    }

    display_completed_plan(hwnd, a, result->task, a.active_request, std::move(result->plan), cache_note);
}

bool try_show_cached_result(HWND hwnd, App& a, TaskKind task, const MissionRequest& in, const CacheKey& key) {
    if (!a.cache.loaded) load_planner_cache(a.cache);

    MissionResult cached;
    if (!find_cached_result(a.cache, key, cached)) return false;

    display_completed_plan(hwnd, a, task, in, std::move(cached), L"[Cache] Loaded cached result.");
    return true;
}

void run_plan(HWND hwnd, App& a, bool force_recompute = false) {
    if (a.busy) return;
    MissionRequest in{};
    std::wstring err;
    if (!read_inputs(a, in, err)) {
        MessageBoxW(hwnd, err.c_str(), L"Input Error", MB_OK | MB_ICONWARNING);
        return;
    }
    const CacheKey key = make_cache_key(CacheTask::PlanMission, in);
    if (!force_recompute && try_show_cached_result(hwnd, a, TaskKind::PlanMission, in, key)) {
        return;
    }

    CacheWriteRequest cache_write;
    cache_write.enabled = true;
    cache_write.forced = force_recompute;
    cache_write.key = key;
    launch_task(hwnd, a, TaskKind::PlanMission, in, cache_write);
}

void run_plan_burnout_no_recovery(HWND hwnd, App& a) {
    if (a.busy) return;
    MissionRequest in{};
    std::wstring err;
    if (!read_inputs(a, in, err)) {
        MessageBoxW(hwnd, err.c_str(), L"Input Error", MB_OK | MB_ICONWARNING);
        return;
    }
    launch_task(hwnd, a, TaskKind::PlanBurnoutNoRecovery, in);
}

void run_max_payload(HWND hwnd, App& a, bool force_recompute = false) {
    if (a.busy) return;
    MissionRequest in{};
    std::wstring err;
    if (!read_inputs(a, in, err)) {
        MessageBoxW(hwnd, err.c_str(), L"Input Error", MB_OK | MB_ICONWARNING);
        return;
    }
    in.payload_kg = 0.0;
    in = falcon9::sanitize_request(in);

    const CacheKey key = make_cache_key(CacheTask::MaxPayload, in);
    if (!force_recompute && try_show_cached_result(hwnd, a, TaskKind::MaxPayload, in, key)) {
        return;
    }

    CacheWriteRequest cache_write;
    cache_write.enabled = true;
    cache_write.forced = force_recompute;
    cache_write.key = key;
    launch_task(hwnd, a, TaskKind::MaxPayload, in, cache_write);
}

struct EarthTexture {
    bool tried = false;
    bool loaded = false;
    int width = 0;
    int height = 0;
    std::vector<uint32_t> bgrx;
};

uint32_t dib_bgrx_from_rgb(int r, int g, int b) {
    r = static_cast<int>(clampd(static_cast<double>(r), 0.0, 255.0));
    g = static_cast<int>(clampd(static_cast<double>(g), 0.0, 255.0));
    b = static_cast<int>(clampd(static_cast<double>(b), 0.0, 255.0));
    return static_cast<uint32_t>(b | (g << 8) | (r << 16));
}

uint32_t dib_bgrx_from_color(COLORREF c) {
    return dib_bgrx_from_rgb(GetRValue(c), GetGValue(c), GetBValue(c));
}

bool find_earth_texture_path(std::filesystem::path& out_path) {
    std::vector<std::filesystem::path> candidates;
    const std::filesystem::path exe_dir = planner_module_dir();
    if (!exe_dir.empty()) {
        candidates.push_back(exe_dir / L"assets" / kEarthTextureFile);
        candidates.push_back(exe_dir / kEarthTextureFile);
        const std::filesystem::path p1 = exe_dir.parent_path();
        if (!p1.empty()) {
            candidates.push_back(p1 / L"assets" / kEarthTextureFile);
            const std::filesystem::path p2 = p1.parent_path();
            if (!p2.empty()) candidates.push_back(p2 / L"assets" / kEarthTextureFile);
        }
    }
    candidates.push_back(std::filesystem::current_path() / L"assets" / kEarthTextureFile);
    candidates.push_back(std::filesystem::current_path() / kEarthTextureFile);

    for (const auto& p : candidates) {
        if (is_regular_file_path(p)) {
            out_path = p;
            return true;
        }
    }
    return false;
}

bool load_earth_texture(EarthTexture& tex) {
    if (tex.tried) return tex.loaded;
    tex.tried = true;

    std::filesystem::path path;
    if (!find_earth_texture_path(path)) return false;

    HBITMAP bmp = reinterpret_cast<HBITMAP>(LoadImageW(
        nullptr,
        path.c_str(),
        IMAGE_BITMAP,
        0,
        0,
        LR_LOADFROMFILE | LR_CREATEDIBSECTION));
    if (!bmp) return false;

    BITMAP bm{};
    if (GetObjectW(bmp, sizeof(bm), &bm) == 0 || bm.bmWidth <= 0 || bm.bmHeight <= 0) {
        DeleteObject(bmp);
        return false;
    }

    BITMAPINFO bi{};
    bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bi.bmiHeader.biWidth = bm.bmWidth;
    bi.bmiHeader.biHeight = -bm.bmHeight;
    bi.bmiHeader.biPlanes = 1;
    bi.bmiHeader.biBitCount = 32;
    bi.bmiHeader.biCompression = BI_RGB;

    std::vector<uint32_t> pixels(static_cast<size_t>(bm.bmWidth) * static_cast<size_t>(bm.bmHeight));
    HDC screen = GetDC(nullptr);
    const int rows = GetDIBits(
        screen,
        bmp,
        0,
        static_cast<UINT>(bm.bmHeight),
        pixels.data(),
        &bi,
        DIB_RGB_COLORS);
    ReleaseDC(nullptr, screen);
    DeleteObject(bmp);
    if (rows != bm.bmHeight) return false;

    tex.width = bm.bmWidth;
    tex.height = bm.bmHeight;
    tex.bgrx = std::move(pixels);
    tex.loaded = true;
    return true;
}

EarthTexture& earth_texture() {
    static EarthTexture tex;
    load_earth_texture(tex);
    return tex;
}

uint32_t sample_earth_texture_bgrx(const EarthTexture& tex, double lat_deg, double lon_deg, double view_depth) {
    if (!tex.loaded || tex.width <= 0 || tex.height <= 0 || tex.bgrx.empty()) {
        return dib_bgrx_from_rgb(38, 74, 109);
    }

    const double lon = wrap_lon_deg(lon_deg);
    double u = (lon + 180.0) / 360.0;
    u = u - std::floor(u);
    const double v = clampd((90.0 - lat_deg) / 180.0, 0.0, 1.0);
    const int x = std::min(tex.width - 1, std::max(0, static_cast<int>(std::floor(u * static_cast<double>(tex.width)))));
    const int y = std::min(tex.height - 1, std::max(0, static_cast<int>(std::floor(v * static_cast<double>(tex.height)))));
    const uint32_t src = tex.bgrx[static_cast<size_t>(y) * static_cast<size_t>(tex.width) + static_cast<size_t>(x)];

    int b = static_cast<int>(src & 0xff);
    int g = static_cast<int>((src >> 8) & 0xff);
    int r = static_cast<int>((src >> 16) & 0xff);
    const double shade = 0.44 + 0.56 * clampd(view_depth, 0.0, 1.0);
    const double haze = std::pow(clampd(1.0 - view_depth, 0.0, 1.0), 2.0) * 0.34;
    r = static_cast<int>(r * shade * (1.0 - haze) + 88.0 * haze);
    g = static_cast<int>(g * shade * (1.0 - haze) + 142.0 * haze);
    b = static_cast<int>(b * shade * (1.0 - haze) + 205.0 * haze);
    return dib_bgrx_from_rgb(r, g, b);
}

void draw_panel_title(HDC hdc, const RECT& outer, const wchar_t* title) {
    SetBkMode(hdc, TRANSPARENT);
    SetTextColor(hdc, kText);
    TextOutW(hdc, outer.left + 10, outer.top + 5, title, lstrlenW(title));
}

void draw_panel_frame(HDC hdc, const RECT& outer) {
    HBRUSH bg = CreateSolidBrush(kPanelBg);
    FillRect(hdc, &outer, bg);
    DeleteObject(bg);

    HPEN bd = CreatePen(PS_SOLID, 1, kBorderOrange);
    HPEN oldp = reinterpret_cast<HPEN>(SelectObject(hdc, bd));
    HBRUSH oldb = reinterpret_cast<HBRUSH>(SelectObject(hdc, GetStockObject(HOLLOW_BRUSH)));
    Rectangle(hdc, outer.left, outer.top, outer.right, outer.bottom);
    SelectObject(hdc, oldb);
    SelectObject(hdc, oldp);
    DeleteObject(bd);
    SetBkMode(hdc, TRANSPARENT);
    SetTextColor(hdc, kText);
}

void draw_profile_panel(HDC hdc, const RECT& outer, const MissionResult& p) {
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
            if (!falcon9::finite_plot_pt(q)) continue;
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
    in.left += 62;
    in.right -= 20;
    in.top += 34;
    in.bottom -= 38;
    if (in.right <= in.left + 10 || in.bottom <= in.top + 10) return;

    const wchar_t* title = L"Trajectory Profile (Altitude km vs Downrange km)";
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
    TextOutW(hdc, (in.left + in.right) / 2 - 48, in.bottom + 24, L"Downrange (km)", 14);

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
        std::vector<POINT> pts;
        pts.reserve(s.pts.size());
        for (const PlotPt& q : s.pts) {
            if (!falcon9::finite_plot_pt(q)) continue;
            pts.push_back(map_pt(q));
        }
        if (pts.size() < 2) continue;
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

void draw_globe_panel(HDC hdc, const RECT& outer, const MissionResult& p, double view_lat_deg, double view_lon_deg, double zoom) {
    draw_panel_frame(hdc, outer);

    RECT header = outer;
    header.left += 1;
    header.top += 1;
    header.right -= 1;
    header.bottom = std::min<LONG>(outer.bottom, outer.top + 28);
    HBRUSH header_brush = CreateSolidBrush(kPanelBg2);
    FillRect(hdc, &header, header_brush);
    DeleteObject(header_brush);

    draw_panel_title(hdc, outer, L"DISPLAY");
    SetTextColor(hdc, kMutedText);
    TextOutW(hdc, outer.right - 360, outer.top + 5, L"Drag to rotate, mouse wheel to zoom", 35);

    RECT in = outer;
    in.left += 14;
    in.right -= 14;
    in.top += 38;
    in.bottom -= 12;
    const int w = in.right - in.left;
    const int h = in.bottom - in.top;
    if (w <= 60 || h <= 60) return;

    const int cx = in.left + w / 2;
    const int cy = in.top + h / 2;
    const double z = clampd(zoom, 0.45, 2.8);
    const int base_radius_px = std::max(10, std::min(w, h) / 2 - 28);
    const int radius_px = std::max(10, static_cast<int>(std::lround(static_cast<double>(base_radius_px) * z)));

    const Vec3 view_n = falcon9::normalize3(falcon9::ecef_from_geo(view_lat_deg, view_lon_deg, 0.0));
    const double view_lon = deg2rad(view_lon_deg);
    const double view_lat = deg2rad(view_lat_deg);
    const Vec3 view_u = falcon9::normalize3({-std::sin(view_lon), std::cos(view_lon), 0.0});
    const Vec3 view_v = falcon9::normalize3({
        -std::sin(view_lat) * std::cos(view_lon),
        -std::sin(view_lat) * std::sin(view_lon),
        std::cos(view_lat)});

    auto project = [&](const Vec3& q, POINT& out, double& depth) {
        depth = falcon9::dot3(falcon9::normalize3(q), view_n);
        out.x = cx + static_cast<int>(std::lround(falcon9::dot3(q, view_u) * radius_px));
        out.y = cy - static_cast<int>(std::lround(falcon9::dot3(q, view_v) * radius_px));
    };

    const int saved_dc = SaveDC(hdc);
    IntersectClipRect(hdc, outer.left + 1, header.bottom, outer.right - 1, outer.bottom - 1);

    RECT globe_rect{cx - radius_px, cy - radius_px, cx + radius_px + 1, cy + radius_px + 1};
    RECT globe_clip{outer.left + 1, header.bottom, outer.right - 1, outer.bottom - 1};
    RECT render_rect{};
    if (!IntersectRect(&render_rect, &globe_rect, &globe_clip)) {
        RestoreDC(hdc, saved_dc);
        return;
    }

    const int globe_w = render_rect.right - render_rect.left;
    const int globe_h = render_rect.bottom - render_rect.top;
    std::vector<uint32_t> globe_pixels(static_cast<size_t>(globe_w) * static_cast<size_t>(globe_h), dib_bgrx_from_color(kPanelBg));
    const EarthTexture& tex = earth_texture();
    for (int py = 0; py < globe_h; ++py) {
        const int dst_y = render_rect.top + py;
        const double sy = static_cast<double>(cy - dst_y) / static_cast<double>(radius_px);
        for (int px = 0; px < globe_w; ++px) {
            const int dst_x = render_rect.left + px;
            const double sx = static_cast<double>(dst_x - cx) / static_cast<double>(radius_px);
            const double rr = sx * sx + sy * sy;
            if (rr > 1.0) continue;
            const double sz = std::sqrt(std::max(0.0, 1.0 - rr));
            const Vec3 q = falcon9::normalize3({
                view_u.x * sx + view_v.x * sy + view_n.x * sz,
                view_u.y * sx + view_v.y * sy + view_n.y * sz,
                view_u.z * sx + view_v.z * sy + view_n.z * sz,
            });
            const double lat = falcon9::rad2deg(std::asin(clampd(q.z, -1.0, 1.0)));
            const double lon = wrap_lon_deg(falcon9::rad2deg(std::atan2(q.y, q.x)));
            globe_pixels[static_cast<size_t>(py) * static_cast<size_t>(globe_w) + static_cast<size_t>(px)] =
                sample_earth_texture_bgrx(tex, lat, lon, sz);
        }
    }

    BITMAPINFO bi{};
    bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bi.bmiHeader.biWidth = globe_w;
    bi.bmiHeader.biHeight = -globe_h;
    bi.bmiHeader.biPlanes = 1;
    bi.bmiHeader.biBitCount = 32;
    bi.bmiHeader.biCompression = BI_RGB;
    StretchDIBits(
        hdc,
        render_rect.left,
        render_rect.top,
        globe_w,
        globe_h,
        0,
        0,
        globe_w,
        globe_h,
        globe_pixels.data(),
        &bi,
        DIB_RGB_COLORS,
        SRCCOPY);

    HBRUSH old_hollow = reinterpret_cast<HBRUSH>(SelectObject(hdc, GetStockObject(HOLLOW_BRUSH)));
    HPEN atmosphere_outer = CreatePen(PS_SOLID, 10, RGB(188, 226, 249));
    HPEN oldp = reinterpret_cast<HPEN>(SelectObject(hdc, atmosphere_outer));
    Ellipse(hdc, cx - radius_px - 5, cy - radius_px - 5, cx + radius_px + 5, cy + radius_px + 5);
    SelectObject(hdc, oldp);
    DeleteObject(atmosphere_outer);

    HPEN atmosphere_inner = CreatePen(PS_SOLID, 2, RGB(78, 168, 232));
    oldp = reinterpret_cast<HPEN>(SelectObject(hdc, atmosphere_inner));
    Ellipse(hdc, cx - radius_px - 1, cy - radius_px - 1, cx + radius_px + 1, cy + radius_px + 1);
    SelectObject(hdc, oldp);
    DeleteObject(atmosphere_inner);
    SelectObject(hdc, old_hollow);

    HPEN globe_border = CreatePen(PS_SOLID, 1, RGB(120, 145, 165));
    oldp = reinterpret_cast<HPEN>(SelectObject(hdc, globe_border));
    HBRUSH oldb = reinterpret_cast<HBRUSH>(SelectObject(hdc, GetStockObject(HOLLOW_BRUSH)));
    Ellipse(hdc, cx - radius_px, cy - radius_px, cx + radius_px, cy + radius_px);
    SelectObject(hdc, oldb);
    SelectObject(hdc, oldp);
    DeleteObject(globe_border);

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
            const Vec3 q = falcon9::ecef_from_geo(lat_deg, lon_deg, 0.0);
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
        draw_geo_line(true, static_cast<double>(lat_deg), RGB(86, 106, 116), 1);
    }
    for (int lon_deg = -150; lon_deg <= 180; lon_deg += 30) {
        draw_geo_line(false, static_cast<double>(lon_deg), RGB(86, 106, 116), 1);
    }

    auto draw_globe_series_line = [&](const GlobeSeries& s) {
        if (s.pts.size() < 2) return;
        HPEN pen = CreatePen(PS_SOLID, 2, s.color);
        HPEN prev_pen = reinterpret_cast<HPEN>(SelectObject(hdc, pen));
        for (size_t i = 1; i < s.pts.size(); ++i) {
            if (!falcon9::finite_globe_pt(s.pts[i - 1]) || !falcon9::finite_globe_pt(s.pts[i])) continue;
            const Vec3 q1 = falcon9::ecef_from_geo(s.pts[i - 1].lat_deg, s.pts[i - 1].lon_deg, s.pts[i - 1].alt_km);
            const Vec3 q2 = falcon9::ecef_from_geo(s.pts[i].lat_deg, s.pts[i].lon_deg, s.pts[i].alt_km);
            POINT p1{};
            POINT p2{};
            double d1 = 0.0;
            double d2 = 0.0;
            project(q1, p1, d1);
            project(q2, p2, d2);
            const double dot12 = falcon9::dot3(
                falcon9::normalize3(q1),
                falcon9::normalize3(q2));
            const int dx = p2.x - p1.x;
            const int dy = p2.y - p1.y;
            const int max_segment_px = std::max(24, radius_px / 2);
            if (d1 >= 0.0 && d2 >= 0.0) {
                if (dot12 < 0.96 || dx * dx + dy * dy > max_segment_px * max_segment_px) continue;
                MoveToEx(hdc, p1.x, p1.y, nullptr);
                LineTo(hdc, p2.x, p2.y);
            }
        }
        SelectObject(hdc, prev_pen);
        DeleteObject(pen);
    };

    for (const GlobeSeries& s : p.globe_series) {
        if (s.name == L"Stage2 Insertion") continue;
        draw_globe_series_line(s);
    }
    for (const GlobeSeries& s : p.globe_series) {
        if (s.name == L"Stage2 Insertion") draw_globe_series_line(s);
    }

    auto draw_site = [&](double lat_deg, double lon_deg, COLORREF color, const wchar_t* tag) {
        const Vec3 q = falcon9::ecef_from_geo(lat_deg, lon_deg, 0.0);
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
        const int tx = pt.x + 6;
        const int ty = pt.y - 8;
        SetTextColor(hdc, RGB(12, 22, 30));
        TextOutW(hdc, tx + 1, ty + 1, tag, lstrlenW(tag));
        SetTextColor(hdc, RGB(250, 252, 255));
        TextOutW(hdc, tx, ty, tag, lstrlenW(tag));
    };

    draw_site(p.launch_lat_deg, p.launch_lon_deg, RGB(52, 73, 94), L"Launch");
    draw_site(p.ship_lat_deg, p.ship_lon_deg, RGB(22, 160, 133), L"Droneship");

    if (p.globe_series.empty()) {
        SetTextColor(hdc, kMutedText);
        const wchar_t* msg = L"Planning output will draw trajectories here.";
        TextOutW(hdc, outer.left + 16, outer.top + 34, msg, lstrlenW(msg));
    } else {
        const int lx = outer.left + 12;
        const int ly0 = outer.top + 34;
        const int legend_w = 260;
        const int legend_h = 10 + static_cast<int>(p.globe_series.size()) * 18;
        RECT legend{lx - 6, ly0 - 5, lx - 6 + legend_w, ly0 - 5 + legend_h};
        HBRUSH legend_bg = CreateSolidBrush(kPanelBg);
        FillRect(hdc, &legend, legend_bg);
        DeleteObject(legend_bg);
        HPEN legend_border = CreatePen(PS_SOLID, 1, RGB(205, 210, 215));
        HPEN old_legend_pen = reinterpret_cast<HPEN>(SelectObject(hdc, legend_border));
        HBRUSH old_legend_brush = reinterpret_cast<HBRUSH>(SelectObject(hdc, GetStockObject(HOLLOW_BRUSH)));
        Rectangle(hdc, legend.left, legend.top, legend.right, legend.bottom);
        SelectObject(hdc, old_legend_brush);
        SelectObject(hdc, old_legend_pen);
        DeleteObject(legend_border);

        SetBkMode(hdc, TRANSPARENT);
        SetTextColor(hdc, kText);
        int ly = ly0;
        for (const GlobeSeries& s : p.globe_series) {
            HPEN lpen = CreatePen(PS_SOLID, 2, s.color);
            HPEN prev_pen = reinterpret_cast<HPEN>(SelectObject(hdc, lpen));
            MoveToEx(hdc, lx, ly + 7, nullptr);
            LineTo(hdc, lx + 24, ly + 7);
            SelectObject(hdc, prev_pen);
            DeleteObject(lpen);
            SetTextColor(hdc, kText);
            TextOutW(hdc, lx + 30, ly, s.name.c_str(), static_cast<int>(s.name.size()));
            ly += 18;
        }
    }

    RestoreDC(hdc, saved_dc);
}

GlobeSeries profile_to_globe_series(const Series& src, const MissionResult& base) {
    GlobeSeries gs;
    gs.name = src.name;
    gs.color = src.color;
    gs.pts.reserve(src.pts.size());
    for (const PlotPt& q : src.pts) {
        if (!falcon9::finite_plot_pt(q)) continue;
        GlobePt gp;
        falcon9::destination_from_course(
            base.launch_lat_deg,
            base.launch_lon_deg,
            base.orbit_target.launch_az_deg,
            std::max(0.0, q.x_km),
            gp.lat_deg,
            gp.lon_deg);
        gp.alt_km = std::max(0.0, q.y_km);
        if (falcon9::finite_globe_pt(gp)) gs.pts.push_back(gp);
    }
    return gs;
}

bool stage2_insertion_display_anchor(const MissionResult& result, GlobePt& anchor, Vec3& horiz) {
    for (const GlobeSeries& series : result.globe_series) {
        if (series.name != L"Stage2 Insertion" || series.pts.size() < 2) continue;

        size_t last_index = series.pts.size();
        while (last_index > 0 && !falcon9::finite_globe_pt(series.pts[last_index - 1])) --last_index;
        if (last_index == 0) continue;

        const GlobePt& last = series.pts[last_index - 1];
        const Vec3 rhat = falcon9::normalize3(falcon9::ecef_from_geo(last.lat_deg, last.lon_deg, 0.0));
        for (size_t prev_index = last_index - 1; prev_index > 0; --prev_index) {
            const GlobePt& prev = series.pts[prev_index - 1];
            if (!falcon9::finite_globe_pt(prev)) continue;
            const Vec3 prev_hat = falcon9::normalize3(falcon9::ecef_from_geo(prev.lat_deg, prev.lon_deg, 0.0));
            const Vec3 raw{
                rhat.x - prev_hat.x,
                rhat.y - prev_hat.y,
                rhat.z - prev_hat.z,
            };
            const double radial_component = falcon9::dot3(raw, rhat);
            const Vec3 tangent{
                raw.x - radial_component * rhat.x,
                raw.y - radial_component * rhat.y,
                raw.z - radial_component * rhat.z,
            };
            if (falcon9::dot3(tangent, tangent) <= 1e-12) continue;
            anchor = last;
            horiz = falcon9::normalize3(tangent);
            return true;
        }
    }
    return false;
}

void append_candidate_post_orbit_series(MissionResult& out) {
    const falcon9::Stage2Result& stage2 = out.stage2;
    if (!stage2.orbit_ok) return;

    GlobePt insertion_gp{};
    falcon9::destination_from_course(
        out.launch_lat_deg,
        out.launch_lon_deg,
        out.orbit_target.launch_az_deg,
        std::max(0.0, stage2.seco.theta * (falcon9::kRe / 1000.0)),
        insertion_gp.lat_deg,
        insertion_gp.lon_deg);
    insertion_gp.alt_km = std::max(0.0, (stage2.seco.r - falcon9::kRe) / 1000.0);

    Vec3 horiz{};
    if (!stage2_insertion_display_anchor(out, insertion_gp, horiz)) {
        const Vec3 east = falcon9::normalize3({-std::sin(deg2rad(insertion_gp.lon_deg)), std::cos(deg2rad(insertion_gp.lon_deg)), 0.0});
        const Vec3 north = falcon9::normalize3({
            -std::sin(deg2rad(insertion_gp.lat_deg)) * std::cos(deg2rad(insertion_gp.lon_deg)),
            -std::sin(deg2rad(insertion_gp.lat_deg)) * std::sin(deg2rad(insertion_gp.lon_deg)),
            std::cos(deg2rad(insertion_gp.lat_deg))});
        const double az_rad = deg2rad(out.orbit_target.launch_az_deg);
        horiz = falcon9::normalize3({
            north.x * std::cos(az_rad) + east.x * std::sin(az_rad),
            north.y * std::cos(az_rad) + east.y * std::sin(az_rad),
            north.z * std::cos(az_rad) + east.z * std::sin(az_rad),
        });
    }
    insertion_gp.alt_km = std::max(0.0, (stage2.seco.r - falcon9::kRe) / 1000.0);
    const Vec3 rhat = falcon9::normalize3(falcon9::ecef_from_geo(insertion_gp.lat_deg, insertion_gp.lon_deg, 0.0));
    const double fpa_rad = deg2rad(stage2.orbit.flight_path_deg);
    const Vec3 vdir = falcon9::normalize3({
        horiz.x * std::cos(fpa_rad) + rhat.x * std::sin(fpa_rad),
        horiz.y * std::cos(fpa_rad) + rhat.y * std::sin(fpa_rad),
        horiz.z * std::cos(fpa_rad) + rhat.z * std::sin(fpa_rad),
    });

    const double rmag_km = stage2.seco.r / 1000.0;
    const double speed_kmps = stage2.orbit.speed_mps / 1000.0;
    const Vec3 r_vec{rmag_km * rhat.x, rmag_km * rhat.y, rmag_km * rhat.z};
    const Vec3 v_vec{speed_kmps * vdir.x, speed_kmps * vdir.y, speed_kmps * vdir.z};
    const Vec3 h_vec = falcon9::cross3(r_vec, v_vec);
    const double h_norm = std::sqrt(falcon9::dot3(h_vec, h_vec));
    if (h_norm <= 1e-8) return;

    const Vec3 h_hat = falcon9::normalize3(h_vec);
    const Vec3 e_vec = {
        (v_vec.y * h_vec.z - v_vec.z * h_vec.y) / falcon9::kMuKm - r_vec.x / std::max(1e-9, rmag_km),
        (v_vec.z * h_vec.x - v_vec.x * h_vec.z) / falcon9::kMuKm - r_vec.y / std::max(1e-9, rmag_km),
        (v_vec.x * h_vec.y - v_vec.y * h_vec.x) / falcon9::kMuKm - r_vec.z / std::max(1e-9, rmag_km),
    };
    const double e_norm = std::sqrt(falcon9::dot3(e_vec, e_vec));
    Vec3 u_orb = (e_norm > 1e-8) ? falcon9::normalize3(e_vec) : falcon9::normalize3(r_vec);
    Vec3 w_orb = falcon9::normalize3(falcon9::cross3(h_hat, u_orb));
    if (falcon9::dot3(w_orb, w_orb) < 1e-10) {
        w_orb = falcon9::normalize3(falcon9::cross3(h_hat, falcon9::normalize3(r_vec)));
    }
    const double nu0 = std::atan2(falcon9::dot3(falcon9::normalize3(r_vec), w_orb), falcon9::dot3(falcon9::normalize3(r_vec), u_orb));
    const double p_orb = (h_norm * h_norm) / falcon9::kMuKm;
    const double e_orb = clampd(stage2.orbit.e, 0.0, 0.99);

    GlobeSeries post_orbit;
    post_orbit.name = L"Post-Insertion Orbit";
    post_orbit.color = RGB(243, 156, 18);
    constexpr int n_orbit = 270;
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
        const double qn = std::sqrt(falcon9::dot3(q, q));
        GlobePt gp{};
        gp.lat_deg = falcon9::rad2deg(std::asin(clampd(q.z / std::max(1e-9, qn), -1.0, 1.0)));
        gp.lon_deg = wrap_lon_deg(falcon9::rad2deg(std::atan2(q.y, q.x)));
        gp.alt_km = std::max(0.0, qn - (falcon9::kRe / 1000.0));
        post_orbit.pts.push_back(gp);
    }
    out.globe_series.push_back(std::move(post_orbit));
}

MissionResult build_candidate_display_plan(const App& a) {
    const SeparationCandidate* cand = selected_candidate(a);
    if (!cand) return a.plan;

    MissionResult out;
    out.status = a.plan.status;
    out.launch_lat_deg = a.plan.launch_lat_deg;
    out.launch_lon_deg = a.plan.launch_lon_deg;
    out.launch_epoch_utc_jd = a.plan.launch_epoch_utc_jd;
    out.ship_lat_deg = a.plan.ship_lat_deg;
    out.ship_lon_deg = a.plan.ship_lon_deg;
    out.view_lat_deg = a.plan.view_lat_deg;
    out.view_lon_deg = a.plan.view_lon_deg;
    out.orbit_target = a.plan.orbit_target;
    out.best_candidate = *cand;
    out.stage1 = cand->stage1;
    out.stage2 = cand->stage2;
    out.recovery = cand->recovery;

    if (std::isfinite(cand->recovery.touchdown_lat_deg) && std::isfinite(cand->recovery.touchdown_lon_deg) &&
        (!cand->recovery.coast_traj.empty() || !cand->recovery.landing_traj.empty())) {
        out.ship_lat_deg = cand->recovery.touchdown_lat_deg;
        out.ship_lon_deg = cand->recovery.touchdown_lon_deg;
    }

    Series ascent;
    ascent.name = L"Stage1 LVD Ascent";
    ascent.color = RGB(41, 128, 185);
    for (const falcon9::SimPt& pt : cand->stage1.traj) ascent.pts.push_back({pt.x_km, pt.z_km});
    out.profile_series.push_back(std::move(ascent));

    if (!cand->recovery.coast_traj.empty()) {
        Series coast;
        coast.name = L"Recovery Coast";
        coast.color = RGB(39, 174, 96);
        for (const falcon9::SimPt& pt : cand->recovery.coast_traj) coast.pts.push_back({pt.x_km, pt.z_km});
        out.profile_series.push_back(std::move(coast));
    }

    if (!cand->recovery.landing_traj.empty()) {
        Series landing;
        landing.name = L"Landing Burn";
        landing.color = RGB(22, 160, 133);
        for (const falcon9::SimPt& pt : cand->recovery.landing_traj) landing.pts.push_back({pt.x_km, pt.z_km});
        out.profile_series.push_back(std::move(landing));
    }

    Series insertion;
    insertion.name = L"Stage2 Insertion";
    insertion.color = RGB(192, 57, 43);
    for (const falcon9::SimPt& pt : cand->stage2.traj) insertion.pts.push_back({pt.x_km, pt.z_km});
    out.profile_series.push_back(std::move(insertion));

    for (const Series& series : out.profile_series) {
        out.globe_series.push_back(profile_to_globe_series(series, out));
    }
    append_candidate_post_orbit_series(out);
    return out;
}

const Series* find_separation_time_series(const MissionResult& p, const wchar_t* name) {
    for (const Series& s : p.separation_time_series) {
        if (s.name == name) return &s;
    }
    return nullptr;
}

void draw_state_line(HDC hdc, int x, int& y, const std::wstring& label, const std::wstring& value) {
    SetTextColor(hdc, kText);
    TextOutW(hdc, x, y, label.c_str(), static_cast<int>(label.size()));
    SetTextColor(hdc, RGB(45, 51, 58));
    TextOutW(hdc, x + 148, y, value.c_str(), static_cast<int>(value.size()));
    y += 14;
}

void draw_spacecraft_state_panel(HDC hdc, const RECT& outer, const App& a, bool final_state) {
    draw_panel_frame(hdc, outer);
    draw_panel_title(hdc, outer, final_state ? L"Final Spacecraft State" : L"Initial Spacecraft State");

    RECT in = outer;
    in.left += 8;
    in.right -= 8;
    in.top += 28;
    in.bottom -= 6;
    if (in.bottom <= in.top + 20) return;

    const MissionRequest req = falcon9::sanitize_request(a.active_request);
    int y = in.top;
    if (!final_state) {
        const double total_mass_kg =
            req.payload_kg +
            req.s1_dry_kg + req.s1_prop_kg +
            req.s2_dry_kg + req.s2_prop_kg;
        draw_state_line(hdc, in.left, y, L"Event", L"Liftoff");
        draw_state_line(hdc, in.left, y, L"UTC", utc_from_jd(req.launch_epoch_utc_jd));
        draw_state_line(hdc, in.left, y, L"Perigee Alt", fnum(req.perigee_km, 1) + L" km");
        draw_state_line(hdc, in.left, y, L"Apogee Alt", fnum(req.apogee_km, 1) + L" km");
        const double direct_incl = falcon9::direct_launch_target_incl_deg(req.lat_deg, req.incl_deg);
        const double direct_az = falcon9::direct_launch_azimuth_deg(req.lat_deg, req.incl_deg);
        draw_state_line(hdc, in.left, y, L"Requested Incl", fnum(req.incl_deg, 3) + L" deg");
        draw_state_line(hdc, in.left, y, L"Direct Plane", fnum(direct_incl, 3) + L" deg @ " + fnum(direct_az, 3) + L" deg");
        draw_state_line(hdc, in.left, y, L"Launch Site", fnum(req.lat_deg, 4) + L", " + fnum(req.launch_lon_deg, 4));
        draw_state_line(hdc, in.left, y, L"Total Mass", fnum(total_mass_kg / 1000.0, 3) + L" t");
        return;
    }

    if (!std::isfinite(a.plan.stage2.cutoff_s) || a.plan.globe_series.empty()) {
        SetTextColor(hdc, kMutedText);
        const wchar_t* msg = L"Final state appears after planning.";
        TextOutW(hdc, in.left, y, msg, lstrlenW(msg));
        return;
    }

    const double final_mass_kg = req.payload_kg + req.s2_dry_kg + std::max(0.0, a.plan.stage2.rem_prop);
    draw_state_line(hdc, in.left, y, L"Event", L"Stage2 SECO");
    draw_state_line(hdc, in.left, y, L"UTC", utc_from_jd(req.launch_epoch_utc_jd + a.plan.stage2.cutoff_s / 86400.0));
    draw_state_line(hdc, in.left, y, L"Perigee Alt", fnum(a.plan.stage2.orbit.rp_km, 1) + L" km");
    draw_state_line(hdc, in.left, y, L"Apogee Alt", fnum(a.plan.stage2.orbit.ra_km, 1) + L" km");
    draw_state_line(
        hdc,
        in.left,
        y,
        L"Inclination",
        fnum(falcon9::direct_launch_effective_incl_deg(req.lat_deg, a.plan.orbit_target.launch_az_deg), 3) + L" deg");
    draw_state_line(hdc, in.left, y, L"Eccentricity", fnum(a.plan.stage2.orbit.e, 6));
    draw_state_line(hdc, in.left, y, L"RAAN", fnum(a.plan.lvd_launch_raan_deg, 3) + L" deg");
    draw_state_line(hdc, in.left, y, L"Total Mass", fnum(final_mass_kg / 1000.0, 3) + L" t");
}

void draw_propellant_time_panel(
    HDC hdc,
    const RECT& outer,
    const MissionResult& p,
    const wchar_t* series_name,
    const wchar_t* title,
    double marker_sep_s,
    const wchar_t* marker_label) {
    draw_panel_frame(hdc, outer);

    const Series* series = find_separation_time_series(p, series_name);
    if (!series || series->pts.empty()) {
        const wchar_t* msg = L"Sweep appears after planning.";
        TextOutW(hdc, outer.left + 16, outer.top + 16, msg, lstrlenW(msg));
        return;
    }

    double xmin = 0.0;
    double xmax = 1.0;
    double ymin = 0.0;
    double ymax = 1.0;
    bool first = true;
    for (const PlotPt& q : series->pts) {
        if (!falcon9::finite_plot_pt(q)) continue;
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
    if (first) return;

    ymin = std::min(0.0, ymin);
    if (xmax <= xmin + 1e-6) xmax = xmin + 1.0;
    if (ymax <= ymin + 1e-6) ymax = ymin + 1.0;

    RECT in = outer;
    in.left += 64;
    in.right -= 18;
    in.top += 34;
    in.bottom -= 36;
    if (in.right <= in.left + 20 || in.bottom <= in.top + 20) return;

    SetBkMode(hdc, TRANSPARENT);
    TextOutW(hdc, outer.left + 10, outer.top + 5, title, lstrlenW(title));

    HPEN axis_pen = CreatePen(PS_SOLID, 1, RGB(110, 110, 110));
    HPEN oldp = reinterpret_cast<HPEN>(SelectObject(hdc, axis_pen));
    MoveToEx(hdc, in.left, in.bottom, nullptr);
    LineTo(hdc, in.right, in.bottom);
    MoveToEx(hdc, in.left, in.top, nullptr);
    LineTo(hdc, in.left, in.bottom);
    SelectObject(hdc, oldp);
    DeleteObject(axis_pen);

    for (int i = 0; i <= 4; ++i) {
        const double u = static_cast<double>(i) / 4.0;
        const int x = in.left + static_cast<int>(std::lround((in.right - in.left) * u));
        MoveToEx(hdc, x, in.bottom, nullptr);
        LineTo(hdc, x, in.bottom + 4);
        const std::wstring t = fnum(xmin + (xmax - xmin) * u, 0);
        TextOutW(hdc, x - 16, in.bottom + 8, t.c_str(), static_cast<int>(t.size()));
    }
    for (int i = 0; i <= 4; ++i) {
        const double u = static_cast<double>(i) / 4.0;
        const int y = in.bottom - static_cast<int>(std::lround((in.bottom - in.top) * u));
        MoveToEx(hdc, in.left - 4, y, nullptr);
        LineTo(hdc, in.left, y);
        const std::wstring t = fnum(ymin + (ymax - ymin) * u, 0);
        TextOutW(hdc, in.left - 50, y - 7, t.c_str(), static_cast<int>(t.size()));
    }
    TextOutW(hdc, (in.left + in.right) / 2 - 52, in.bottom + 22, L"Separation Time (s)", 19);

    auto map_pt = [&](const PlotPt& q) {
        POINT p2{};
        const double nx = (q.x_km - xmin) / (xmax - xmin);
        const double ny = (q.y_km - ymin) / (ymax - ymin);
        p2.x = in.left + static_cast<int>(std::lround(nx * (in.right - in.left)));
        p2.y = in.bottom - static_cast<int>(std::lround(ny * (in.bottom - in.top)));
        return p2;
    };

    std::vector<POINT> pts;
    pts.reserve(series->pts.size());
    for (const PlotPt& q : series->pts) {
        if (falcon9::finite_plot_pt(q)) pts.push_back(map_pt(q));
    }
    if (pts.size() >= 2) {
        HPEN line_pen = CreatePen(PS_SOLID, 2, series->color);
        oldp = reinterpret_cast<HPEN>(SelectObject(hdc, line_pen));
        Polyline(hdc, pts.data(), static_cast<int>(pts.size()));
        SelectObject(hdc, oldp);
        DeleteObject(line_pen);
    }

    HBRUSH dot_brush = CreateSolidBrush(series->color);
    HBRUSH oldb = reinterpret_cast<HBRUSH>(SelectObject(hdc, dot_brush));
    HPEN dot_pen = CreatePen(PS_SOLID, 1, series->color);
    oldp = reinterpret_cast<HPEN>(SelectObject(hdc, dot_pen));
    for (const POINT& pt : pts) {
        Ellipse(hdc, pt.x - 2, pt.y - 2, pt.x + 3, pt.y + 3);
    }
    SelectObject(hdc, oldp);
    SelectObject(hdc, oldb);
    DeleteObject(dot_pen);
    DeleteObject(dot_brush);

    if (std::isfinite(marker_sep_s) &&
        marker_sep_s >= xmin &&
        marker_sep_s <= xmax) {
        const double nx = (marker_sep_s - xmin) / (xmax - xmin);
        const int x = in.left + static_cast<int>(std::lround(nx * (in.right - in.left)));
        HPEN marker_pen = CreatePen(PS_DOT, 1, RGB(70, 70, 70));
        oldp = reinterpret_cast<HPEN>(SelectObject(hdc, marker_pen));
        MoveToEx(hdc, x, in.top, nullptr);
        LineTo(hdc, x, in.bottom);
        SelectObject(hdc, oldp);
        DeleteObject(marker_pen);
        const int label_x = (x > in.right - 38) ? (x - 36) : (x + 4);
        TextOutW(hdc, label_x, in.top + 4, marker_label, lstrlenW(marker_label));
    }
}

void draw_plot(HDC hdc, const RECT& outer, const MissionResult& p, double view_lat_deg, double view_lon_deg, double zoom) {
    HBRUSH bg = CreateSolidBrush(kUiBg);
    FillRect(hdc, &outer, bg);
    DeleteObject(bg);

    draw_globe_panel(hdc, outer, p, view_lat_deg, view_lon_deg, zoom);
}

void draw_sweep_charts(HDC hdc, const RECT& outer, const App& a) {
    HBRUSH bg = CreateSolidBrush(kUiBg);
    FillRect(hdc, &outer, bg);
    DeleteObject(bg);

    RECT in = outer;
    in.left += 10;
    in.right -= 10;
    in.top += 10;
    in.bottom -= 10;
    if (in.right <= in.left + 60 || in.bottom <= in.top + 60) return;

    RECT left{};
    RECT middle{};
    RECT right{};
    split_plot_rects_three(in, left, middle, right);

    double marker_sep_s = std::numeric_limits<double>::quiet_NaN();
    if (const SeparationCandidate* cand = selected_candidate(a)) marker_sep_s = cand->sep_time_s;

    draw_propellant_time_panel(
        hdc,
        left,
        a.plan,
        L"Stage2 Remaining Propellant",
        L"S2 Remaining Fuel (kg)",
        marker_sep_s,
        L"Selected");
    draw_propellant_time_panel(
        hdc,
        middle,
        a.plan,
        L"Landing Margin",
        L"Landing Margin (kg)",
        marker_sep_s,
        L"Selected");
    draw_propellant_time_panel(
        hdc,
        right,
        a.plan,
        L"Landing Propellant",
        L"Landing Propellant (kg)",
        marker_sep_s,
        L"Selected");
}

const Series* find_lvd_time_series(const MissionResult& p, const wchar_t* name) {
    for (const Series& s : p.lvd_time_series) {
        if (s.name == name) return &s;
    }
    return nullptr;
}

void draw_series_panel(
    HDC hdc,
    const RECT& outer,
    const Series* series,
    const wchar_t* title,
    const wchar_t* x_label,
    double marker_x,
    const wchar_t* marker_label) {
    draw_panel_frame(hdc, outer);
    SetBkMode(hdc, TRANSPARENT);
    TextOutW(hdc, outer.left + 10, outer.top + 5, title, lstrlenW(title));

    if (!series || series->pts.empty()) {
        const wchar_t* msg = L"No data.";
        TextOutW(hdc, outer.left + 16, outer.top + 28, msg, lstrlenW(msg));
        return;
    }

    double xmin = 0.0;
    double xmax = 1.0;
    double ymin = 0.0;
    double ymax = 1.0;
    bool first = true;
    for (const PlotPt& q : series->pts) {
        if (!falcon9::finite_plot_pt(q)) continue;
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
    if (first) return;
    if (ymin > 0.0) ymin = 0.0;
    if (xmax <= xmin + 1e-6) xmax = xmin + 1.0;
    if (ymax <= ymin + 1e-6) ymax = ymin + 1.0;

    RECT in = outer;
    in.left += 58;
    in.right -= 16;
    in.top += 34;
    in.bottom -= 34;
    if (in.right <= in.left + 20 || in.bottom <= in.top + 20) return;

    HPEN axis_pen = CreatePen(PS_SOLID, 1, RGB(110, 110, 110));
    HPEN oldp = reinterpret_cast<HPEN>(SelectObject(hdc, axis_pen));
    MoveToEx(hdc, in.left, in.bottom, nullptr);
    LineTo(hdc, in.right, in.bottom);
    MoveToEx(hdc, in.left, in.top, nullptr);
    LineTo(hdc, in.left, in.bottom);
    SelectObject(hdc, oldp);
    DeleteObject(axis_pen);

    for (int i = 0; i <= 4; ++i) {
        const double u = static_cast<double>(i) / 4.0;
        const int x = in.left + static_cast<int>(std::lround((in.right - in.left) * u));
        const std::wstring t = fnum(xmin + (xmax - xmin) * u, 0);
        TextOutW(hdc, x - 16, in.bottom + 7, t.c_str(), static_cast<int>(t.size()));
        const int y = in.bottom - static_cast<int>(std::lround((in.bottom - in.top) * u));
        const std::wstring ytxt = fnum(ymin + (ymax - ymin) * u, 0);
        TextOutW(hdc, in.left - 48, y - 7, ytxt.c_str(), static_cast<int>(ytxt.size()));
    }
    TextOutW(hdc, (in.left + in.right) / 2 - 34, in.bottom + 20, x_label, lstrlenW(x_label));

    auto map_pt = [&](const PlotPt& q) {
        POINT p2{};
        p2.x = in.left + static_cast<int>(std::lround(((q.x_km - xmin) / (xmax - xmin)) * (in.right - in.left)));
        p2.y = in.bottom - static_cast<int>(std::lround(((q.y_km - ymin) / (ymax - ymin)) * (in.bottom - in.top)));
        return p2;
    };

    std::vector<POINT> pts;
    pts.reserve(series->pts.size());
    for (const PlotPt& q : series->pts) {
        if (falcon9::finite_plot_pt(q)) pts.push_back(map_pt(q));
    }
    if (pts.size() >= 2) {
        HPEN pen = CreatePen(PS_SOLID, 2, series->color);
        oldp = reinterpret_cast<HPEN>(SelectObject(hdc, pen));
        Polyline(hdc, pts.data(), static_cast<int>(pts.size()));
        SelectObject(hdc, oldp);
        DeleteObject(pen);
    }

    if (std::isfinite(marker_x) && marker_x >= xmin && marker_x <= xmax) {
        const int x = in.left + static_cast<int>(std::lround(((marker_x - xmin) / (xmax - xmin)) * (in.right - in.left)));
        HPEN marker_pen = CreatePen(PS_DOT, 1, RGB(80, 80, 80));
        oldp = reinterpret_cast<HPEN>(SelectObject(hdc, marker_pen));
        MoveToEx(hdc, x, in.top, nullptr);
        LineTo(hdc, x, in.bottom);
        SelectObject(hdc, oldp);
        DeleteObject(marker_pen);
        TextOutW(hdc, x + 4, in.top + 4, marker_label, lstrlenW(marker_label));
    }
}

void draw_lvd_events_panel(HDC hdc, const RECT& outer, const MissionResult& p) {
    draw_panel_frame(hdc, outer);
    SetBkMode(hdc, TRANSPARENT);
    TextOutW(hdc, outer.left + 10, outer.top + 5, L"LVD Events", 10);
    if (p.lvd_events.empty()) {
        TextOutW(hdc, outer.left + 16, outer.top + 30, L"No LVD events.", 14);
        return;
    }

    int y = outer.top + 30;
    TextOutW(hdc, outer.left + 12, y, L"Event", 5);
    TextOutW(hdc, outer.left + 170, y, L"T(s)", 4);
    TextOutW(hdc, outer.left + 235, y, L"Alt", 3);
    TextOutW(hdc, outer.left + 300, y, L"Speed", 5);
    TextOutW(hdc, outer.left + 372, y, L"FPA", 3);
    TextOutW(hdc, outer.left + 430, y, L"Q", 1);
    y += 20;

    for (const falcon9::LvdEvent& ev : p.lvd_events) {
        if (y > outer.bottom - 20) break;
        TextOutW(hdc, outer.left + 12, y, ev.name.c_str(), static_cast<int>(std::min<size_t>(ev.name.size(), 22)));
        const std::wstring t = fnum(ev.t_s, 1);
        const std::wstring alt = fnum(ev.alt_km, 1);
        const std::wstring speed = fnum(ev.speed_mps, 0);
        const std::wstring fpa = fnum(ev.flight_path_deg, 1);
        const std::wstring q = fnum(ev.q_kpa, 1);
        TextOutW(hdc, outer.left + 170, y, t.c_str(), static_cast<int>(t.size()));
        TextOutW(hdc, outer.left + 235, y, alt.c_str(), static_cast<int>(alt.size()));
        TextOutW(hdc, outer.left + 300, y, speed.c_str(), static_cast<int>(speed.size()));
        TextOutW(hdc, outer.left + 372, y, fpa.c_str(), static_cast<int>(fpa.size()));
        TextOutW(hdc, outer.left + 430, y, q.c_str(), static_cast<int>(q.size()));
        y += 18;
    }
}

Series make_launch_window_error_series(const MissionResult& p) {
    Series s;
    s.name = L"Launch Window Plane Error";
    s.color = RGB(52, 152, 219);
    s.pts.reserve(p.launch_window_samples.size());
    for (const falcon9::LaunchWindowSample& w : p.launch_window_samples) {
        s.pts.push_back({w.offset_s / 60.0, std::abs(w.plane_error_deg)});
    }
    return s;
}

void draw_lvd_display(HDC hdc, const RECT& outer, const App& a) {
    HBRUSH bg = CreateSolidBrush(kUiBg);
    FillRect(hdc, &outer, bg);
    DeleteObject(bg);

    RECT in = outer;
    in.left += 10;
    in.right -= 10;
    in.top += 10;
    in.bottom -= 10;
    if (in.right <= in.left + 80 || in.bottom <= in.top + 80) return;

    RECT top = in;
    const int available_h = in.bottom - in.top;
    top.bottom = in.top + std::min<LONG>(available_h, std::max<LONG>(120, available_h / 2));
    RECT bottom = in;
    bottom.top = top.bottom + 8;

    RECT events = top;
    RECT window = top;
    const int split = top.left + (top.right - top.left) / 2;
    events.right = split - 4;
    window.left = split + 4;

    draw_lvd_events_panel(hdc, events, a.plan);
    const Series window_series = make_launch_window_error_series(a.plan);
    draw_series_panel(
        hdc,
        window,
        &window_series,
        L"Launch Window Plane Error (deg)",
        L"Offset min",
        a.plan.lvd_launch_offset_s / 60.0,
        L"LVD");

    if (available_h < 300 || bottom.bottom <= bottom.top + 70) return;

    RECT p1{};
    RECT p2{};
    RECT p3{};
    split_plot_rects_three(bottom, p1, p2, p3);
    draw_series_panel(
        hdc,
        p1,
        find_lvd_time_series(a.plan, L"LVD Altitude (km)"),
        L"Altitude",
        L"Time s",
        a.plan.stage1.sep_s,
        L"SEP");
    draw_series_panel(
        hdc,
        p2,
        find_lvd_time_series(a.plan, L"LVD Speed (m/s)"),
        L"Speed",
        L"Time s",
        a.plan.stage1.sep_s,
        L"SEP");
    draw_series_panel(
        hdc,
        p3,
        find_lvd_time_series(a.plan, L"LVD Dynamic Pressure (kPa)"),
        L"Dynamic Pressure",
        L"Time s",
        a.plan.stage1.t_max_q,
        L"Max-Q");
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
            a->bg_brush = CreateSolidBrush(kUiBg);
            a->panel_brush = CreateSolidBrush(kPanelBg);
            a->base_input = MissionRequest{};
            a->has_vehicle_config = g_startup_vehicle.has_config;
            a->vehicle_config_path = g_startup_vehicle.config_path;
            a->vehicle_config_error = g_startup_vehicle.error;
            if (g_startup_vehicle.has_config) a->base_input = g_startup_vehicle.input;
            load_planner_cache(a->cache);
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
            a->btn_plan_force = CreateWindowExW(0, L"BUTTON", L"Force Recompute", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 100, 30, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnPlanForce)), nullptr, nullptr);
            a->btn_plan_burnout_no_recovery = CreateWindowExW(0, L"BUTTON", L"Plan No-Recovery Burnout", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 100, 30, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnPlanBurnoutNoRecovery)), nullptr, nullptr);
            a->btn_max_payload = CreateWindowExW(0, L"BUTTON", L"Compute Max Payload", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 100, 30, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnMaxPayload)), nullptr, nullptr);
            a->btn_max_payload_force = CreateWindowExW(0, L"BUTTON", L"Force Recompute", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 100, 30, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnMaxPayloadForce)), nullptr, nullptr);
            a->btn_import_default_cfg = CreateWindowExW(0, L"BUTTON", L"Import Default Falcon9 Config", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 100, 30, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnImportDefaultCfg)), nullptr, nullptr);
            a->btn_prev_candidate = CreateWindowExW(0, L"BUTTON", L"<", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 48, 28, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnPrevCandidate)), nullptr, nullptr);
            a->btn_next_candidate = CreateWindowExW(0, L"BUTTON", L">", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 48, 28, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnNextCandidate)), nullptr, nullptr);
            a->candidate_label = CreateWindowExW(0, L"STATIC", L"Candidates: none", WS_CHILD | WS_VISIBLE, 0, 0, 200, 24, hwnd, nullptr, nullptr, nullptr);
            a->list = CreateWindowExW(WS_EX_CLIENTEDGE, L"LISTBOX", L"", WS_CHILD | WS_VISIBLE | WS_VSCROLL | LBS_NOINTEGRALHEIGHT, 0, 0, 200, 100, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kList)), nullptr, nullptr);
            set_font(a->btn, a->font);
            set_font(a->btn_plan_force, a->font);
            set_font(a->btn_plan_burnout_no_recovery, a->font);
            set_font(a->btn_max_payload, a->font);
            set_font(a->btn_max_payload_force, a->font);
            set_font(a->btn_import_default_cfg, a->font);
            set_font(a->btn_prev_candidate, a->font);
            set_font(a->btn_next_candidate, a->font);
            set_font(a->candidate_label, a->font);
            set_font(a->list, a->font);
            sync_candidate_controls(*a);
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
        case WM_ERASEBKGND:
            return 1;
        case WM_CTLCOLORSTATIC:
        case WM_CTLCOLOREDIT:
        case WM_CTLCOLORLISTBOX: {
            App* a = app(hwnd);
            HDC child_dc = reinterpret_cast<HDC>(wp);
            SetTextColor(child_dc, kText);
            SetBkColor(child_dc, kPanelBg);
            return reinterpret_cast<LRESULT>((a && a->panel_brush) ? a->panel_brush : GetStockObject(BLACK_BRUSH));
        }
        case WM_COMMAND: {
            App* a = app(hwnd);
            if (a && LOWORD(wp) == kBtnPlan && HIWORD(wp) == BN_CLICKED) {
                run_plan(hwnd, *a);
                return 0;
            }
            if (a && LOWORD(wp) == kBtnPlanForce && HIWORD(wp) == BN_CLICKED) {
                run_plan(hwnd, *a, true);
                return 0;
            }
            if (a && LOWORD(wp) == kBtnPlanBurnoutNoRecovery && HIWORD(wp) == BN_CLICKED) {
                run_plan_burnout_no_recovery(hwnd, *a);
                return 0;
            }
            if (a && LOWORD(wp) == kBtnPrevCandidate && HIWORD(wp) == BN_CLICKED) {
                if (a->selected_candidate_index > 0) --a->selected_candidate_index;
                sync_candidate_controls(*a);
                InvalidateRect(hwnd, nullptr, TRUE);
                return 0;
            }
            if (a && LOWORD(wp) == kBtnNextCandidate && HIWORD(wp) == BN_CLICKED) {
                if (a->selected_candidate_index + 1 < a->plan.separation_candidates.size()) ++a->selected_candidate_index;
                sync_candidate_controls(*a);
                InvalidateRect(hwnd, nullptr, TRUE);
                return 0;
            }
            if (a && LOWORD(wp) == kBtnMaxPayload && HIWORD(wp) == BN_CLICKED) {
                run_max_payload(hwnd, *a);
                return 0;
            }
            if (a && LOWORD(wp) == kBtnMaxPayloadForce && HIWORD(wp) == BN_CLICKED) {
                run_max_payload(hwnd, *a, true);
                return 0;
            }
            if (a && LOWORD(wp) == kBtnImportDefaultCfg && HIWORD(wp) == BN_CLICKED) {
                std::wstring cfg_path;
                if (!find_default_vehicle_config_path(cfg_path)) {
                    MessageBoxW(hwnd, L"Default Falcon9 config file not found.\nExpected file name: falcon9_real_defaults.txt", L"Config Not Found", MB_OK | MB_ICONWARNING);
                    return 0;
                }
                std::wstring err;
                if (!load_vehicle_config_into_app(*a, cfg_path, err)) {
                    MessageBoxW(hwnd, (std::wstring(L"Failed to load default config:\n") + cfg_path + L"\n\n" + err).c_str(), L"Config Load Error", MB_OK | MB_ICONWARNING);
                    return 0;
                }
                run_plan(hwnd, *a);
                return 0;
            }
            return 0;
        }
        case kMsgWorkerDone: {
            App* a = app(hwnd);
            std::unique_ptr<WorkerResult> result(reinterpret_cast<WorkerResult*>(lp));
            if (a && result) {
                finish_task(hwnd, *a, std::move(result));
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
        case WM_MOUSEWHEEL: {
            App* a = app(hwnd);
            if (!a) return 0;
            POINT pt{GET_X_LPARAM(lp), GET_Y_LPARAM(lp)};
            ScreenToClient(hwnd, &pt);
            if (!PtInRect(&a->globe_panel, pt)) return 0;
            const short wheel_delta = GET_WHEEL_DELTA_WPARAM(wp);
            const double factor = (wheel_delta > 0) ? 1.12 : 1.0 / 1.12;
            a->globe_zoom = clampd(a->globe_zoom * factor, 0.45, 2.8);
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
            if (a) {
                RECT rc{};
                GetClientRect(hwnd, &rc);
                const int w = rc.right - rc.left;
                const int h = rc.bottom - rc.top;
                HDC mem_dc = (w > 0 && h > 0) ? CreateCompatibleDC(hdc) : nullptr;
                HBITMAP mem_bmp = mem_dc ? CreateCompatibleBitmap(hdc, w, h) : nullptr;
                HGDIOBJ old_bmp = mem_bmp ? SelectObject(mem_dc, mem_bmp) : nullptr;
                HDC paint_dc = (mem_dc && mem_bmp) ? mem_dc : hdc;

                HBRUSH bg = CreateSolidBrush(kUiBg);
                FillRect(paint_dc, &rc, bg);
                DeleteObject(bg);
                const MissionResult display_plan = build_candidate_display_plan(*a);
                draw_spacecraft_state_panel(paint_dc, a->initial_state_panel, *a, false);
                draw_spacecraft_state_panel(paint_dc, a->final_state_panel, *a, true);
                draw_plot(paint_dc, a->plot, display_plan, a->view_lat_deg, a->view_lon_deg, a->globe_zoom);

                if (mem_dc && mem_bmp) {
                    BitBlt(
                        hdc,
                        ps.rcPaint.left,
                        ps.rcPaint.top,
                        ps.rcPaint.right - ps.rcPaint.left,
                        ps.rcPaint.bottom - ps.rcPaint.top,
                        mem_dc,
                        ps.rcPaint.left,
                        ps.rcPaint.top,
                        SRCCOPY);
                    SelectObject(mem_dc, old_bmp);
                    DeleteObject(mem_bmp);
                    DeleteDC(mem_dc);
                }
            }
            EndPaint(hwnd, &ps);
            return 0;
        }
        case WM_CLOSE: {
            App* a = app(hwnd);
            if (a) {
                a->closing = true;
                a->cancel_requested.store(true, std::memory_order_relaxed);
                join_worker_if_needed(*a);
                discard_worker_messages(hwnd);
            }
            DestroyWindow(hwnd);
            return 0;
        }
        case WM_DESTROY: {
            App* a = app(hwnd);
            if (a) {
                a->cancel_requested.store(true, std::memory_order_relaxed);
                join_worker_if_needed(*a);
                discard_worker_messages(hwnd);
                if (a->bg_brush) DeleteObject(a->bg_brush);
                if (a->panel_brush) DeleteObject(a->panel_brush);
            }
            delete a;
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, 0);
            PostQuitMessage(0);
            return 0;
        }
        default:
            return DefWindowProcW(hwnd, msg, wp, lp);
    }
}

}  // namespace

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
            if (cfg_path.empty()) {
                std::wstring auto_cfg;
                if (find_default_vehicle_config_path(auto_cfg)) cfg_path = auto_cfg;
            }
            if (!cfg_path.empty()) {
                MissionRequest loaded{};
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

    WNDCLASSW wc{};
    wc.lpfnWndProc = wndproc;
    wc.hInstance = hi;
    wc.lpszClassName = kMainWindowClass;
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    wc.hbrBackground = CreateSolidBrush(kUiBg);
    if (!RegisterClassW(&wc)) return 1;

    HWND hwnd = CreateWindowExW(
        0,
        kMainWindowClass,
        L"Falcon9 Launch Vehicle Designer",
        WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN,
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
