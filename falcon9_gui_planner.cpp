#define NOMINMAX
#include <windows.h>
#include <commctrl.h>
#include <windowsx.h>

#include "planner_mission.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cwchar>
#include <cwctype>
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
constexpr int kBtnPlanCoarse1s = 1005;
constexpr int kBtnPlanBurnoutNoRecovery = 1006;
constexpr int kBtnPrevCandidate = 1007;
constexpr int kBtnNextCandidate = 1008;
constexpr int kFieldBase = 1100;
constexpr wchar_t kDefaultVehicleConfigFile[] = L"falcon9_real_defaults.txt";
constexpr UINT kMsgWorkerDone = WM_APP + 1;
constexpr wchar_t kMainWindowClass[] = L"Falcon9PlannerWinClass";
constexpr wchar_t kSweepWindowClass[] = L"Falcon9SweepChartsWinClass";

enum class TaskKind {
    PlanMission,
    PlanCoarse1s,
    PlanBurnoutNoRecovery,
    MaxPayload,
};

struct WorkerResult {
    TaskKind task = TaskKind::PlanMission;
    MissionResult plan;
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
    HWND btn_plan_coarse_1s = nullptr;
    HWND btn_plan_burnout_no_recovery = nullptr;
    HWND btn_max_payload = nullptr;
    HWND btn_import_default_cfg = nullptr;
    HWND btn_prev_candidate = nullptr;
    HWND btn_next_candidate = nullptr;
    HWND candidate_label = nullptr;
    HWND list = nullptr;
    HWND sweep_hwnd = nullptr;
    HFONT font = nullptr;
    RECT plot{0, 0, 0, 0};
    RECT globe_panel{0, 0, 0, 0};
    bool view_initialized = false;
    double view_lat_deg = 20.0;
    double view_lon_deg = -60.0;
    bool dragging_globe = false;
    POINT last_mouse{0, 0};
    MissionRequest base_input;
    bool has_vehicle_config = false;
    std::wstring vehicle_config_path;
    std::wstring vehicle_config_error;
    MissionResult plan;
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
    MoveWindow(a.btn_plan_coarse_1s, x0, y + 44, left_w - 24, 30, TRUE);
    MoveWindow(a.btn_plan_burnout_no_recovery, x0, y + 80, left_w - 24, 30, TRUE);
    MoveWindow(a.btn_max_payload, x0, y + 116, left_w - 24, 30, TRUE);
    MoveWindow(a.btn_import_default_cfg, x0, y + 152, left_w - 24, 30, TRUE);
    MoveWindow(a.btn_prev_candidate, x0, y + 192, 48, 28, TRUE);
    MoveWindow(a.btn_next_candidate, x0 + 56, y + 192, 48, 28, TRUE);
    MoveWindow(a.candidate_label, x0 + 116, y + 196, left_w - 140, 24, TRUE);

    const int pl = left_w + 16;
    const int pt = 14;
    const int pr = std::max(pl + 240, static_cast<int>(rc.right) - 14);
    const int pb = std::max(pt + 240, static_cast<int>(rc.bottom) - 230);
    a.plot = {pl, pt, pr, pb};
    RECT left{};
    RECT right{};
    split_plot_rects(a.plot, left, right);
    a.globe_panel = right;

    MoveWindow(a.list, pl, pb + 10, std::max(50, pr - pl), std::max(50, static_cast<int>(rc.bottom) - (pb + 24)), TRUE);
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
    EnableWindow(a.btn_plan_coarse_1s, enabled ? TRUE : FALSE);
    EnableWindow(a.btn_plan_burnout_no_recovery, enabled ? TRUE : FALSE);
    EnableWindow(a.btn_max_payload, enabled ? TRUE : FALSE);
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
    } else if (task == TaskKind::PlanCoarse1s) {
        a.plan.lines.push_back(L"[Status] Computing coarse 1s separation-time search...");
    } else if (task == TaskKind::PlanBurnoutNoRecovery) {
        a.plan.lines.push_back(L"[Status] Computing no-recovery burnout separation plan...");
    } else {
        a.plan.lines.push_back(L"[Status] Computing...");
    }
}

MissionResult solve_plan_request(const MissionRequest& in, std::atomic<bool>* cancel_requested) {
    return falcon9::solve_mission(in, {cancel_requested, 0});
}

MissionResult solve_plan_coarse_1s_request(const MissionRequest& in, std::atomic<bool>* cancel_requested) {
    return falcon9::solve_mission(in, {cancel_requested, 0, falcon9::SeparationSearchMode::Coarse1s});
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
           result.recovery.margin_kg > 0.0;
}

MissionResult solve_max_payload_request(const MissionRequest& in, std::atomic<bool>* cancel_requested) {
    constexpr double kMarginTolKg = 5.0;
    constexpr double kPayloadSearchCapKg = 30000.0;
    const double coarse_steps[] = {4000.0, 1000.0, 200.0, 50.0};
    constexpr double kFinalStepKg = 10.0;
    const double payload_cap_kg = std::max(kPayloadSearchCapKg, in.payload_kg);
    bool hit_search_cap = false;

    auto eval_payload = [&](double payload_kg) {
        MissionRequest test = in;
        test.payload_kg = std::max(0.0, payload_kg);
        return falcon9::solve_mission(test, {cancel_requested, 0});
    };

    MissionResult plan_zero = eval_payload(0.0);
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
        while (!cancel_requested->load(std::memory_order_relaxed)) {
            const double raw_probe_payload_kg = best_payload_kg + step_kg;
            const double probe_payload_kg = std::min(raw_probe_payload_kg, payload_cap_kg);
            const bool capped_probe = raw_probe_payload_kg > payload_cap_kg + 1e-6;
            MissionResult probe = eval_payload(probe_payload_kg);
            if (!payload_margin_search_ok(probe)) {
                first_fail_payload_kg = probe_payload_kg;
                break;
            }

            have_positive_margin = true;
            best_payload_kg = probe_payload_kg;
            best_plan = std::move(probe);
            if (capped_probe) {
                hit_search_cap = true;
                break;
            }
            if (best_plan.recovery.margin_kg > kMarginTolKg) continue;
            break;
        }
        if (hit_search_cap) break;
    }

    while (!cancel_requested->load(std::memory_order_relaxed)) {
        const double raw_probe_payload_kg = best_payload_kg + kFinalStepKg;
        const double probe_payload_kg = std::min(raw_probe_payload_kg, payload_cap_kg);
        const bool capped_probe = raw_probe_payload_kg > payload_cap_kg + 1e-6;
        MissionResult probe = eval_payload(probe_payload_kg);
        if (!payload_margin_search_ok(probe)) {
            first_fail_payload_kg = probe_payload_kg;
            break;
        }
        have_positive_margin = true;
        best_payload_kg = probe_payload_kg;
        best_plan = std::move(probe);
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
    best_plan.lines.insert(best_plan.lines.begin() + 1, L"[Payload Capability] Search start payload = 0.0 kg");
    best_plan.lines.insert(best_plan.lines.begin() + 2, L"[Payload Capability] Landing margin at max payload = " + fnum(best_plan.recovery.margin_kg, 1) + L" kg");
    best_plan.lines.insert(best_plan.lines.begin() + 3, L"[Payload Capability] Feasible interval: [" + fnum(best_payload_kg, 1) + L", " + fnum(first_fail_payload_kg, 1) + L"] kg");
    if (hit_search_cap) {
        best_plan.lines.insert(best_plan.lines.begin() + 4, L"[Payload Capability] Search cap reached before landing margin crossed zero.");
    }
    return best_plan;
}

void launch_task(HWND hwnd, App& a, TaskKind task, const MissionRequest& in) {
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
    InvalidateRect(hwnd, &a.plot, TRUE);
    if (a.sweep_hwnd) InvalidateRect(a.sweep_hwnd, nullptr, TRUE);

    try {
        a.worker = std::thread([hwnd, task, in, cancel_requested = &a.cancel_requested]() {
            std::unique_ptr<WorkerResult> result = std::make_unique<WorkerResult>();
            result->task = task;
            if (task == TaskKind::MaxPayload) {
                result->plan = solve_max_payload_request(in, cancel_requested);
            } else if (task == TaskKind::PlanCoarse1s) {
                result->plan = solve_plan_coarse_1s_request(in, cancel_requested);
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
        InvalidateRect(hwnd, &a.plot, TRUE);
    }
}

void finish_task(HWND hwnd, App& a, std::unique_ptr<WorkerResult> result) {
    join_worker_if_needed(a);
    a.busy = false;
    a.cancel_requested.store(false, std::memory_order_relaxed);
    a.plan = std::move(result->plan);
    prepend_vehicle_config_note(a);
    select_best_candidate_index(a);
    set_controls_enabled(a, true);
    sync_candidate_controls(a);
    if (!a.view_initialized) {
        a.view_lat_deg = a.plan.view_lat_deg;
        a.view_lon_deg = a.plan.view_lon_deg;
        a.view_initialized = true;
    }
    refresh_list(a);
    InvalidateRect(hwnd, &a.plot, TRUE);
    if (a.sweep_hwnd) {
        ShowWindow(a.sweep_hwnd, SW_SHOWNOACTIVATE);
        InvalidateRect(a.sweep_hwnd, nullptr, TRUE);
    }
}

void run_plan(HWND hwnd, App& a) {
    MissionRequest in{};
    std::wstring err;
    if (!read_inputs(a, in, err)) {
        MessageBoxW(hwnd, err.c_str(), L"Input Error", MB_OK | MB_ICONWARNING);
        return;
    }
    launch_task(hwnd, a, TaskKind::PlanMission, in);
}

void run_plan_coarse_1s(HWND hwnd, App& a) {
    MissionRequest in{};
    std::wstring err;
    if (!read_inputs(a, in, err)) {
        MessageBoxW(hwnd, err.c_str(), L"Input Error", MB_OK | MB_ICONWARNING);
        return;
    }
    launch_task(hwnd, a, TaskKind::PlanCoarse1s, in);
}

void run_plan_burnout_no_recovery(HWND hwnd, App& a) {
    MissionRequest in{};
    std::wstring err;
    if (!read_inputs(a, in, err)) {
        MessageBoxW(hwnd, err.c_str(), L"Input Error", MB_OK | MB_ICONWARNING);
        return;
    }
    launch_task(hwnd, a, TaskKind::PlanBurnoutNoRecovery, in);
}

void run_max_payload(HWND hwnd, App& a) {
    MissionRequest in{};
    std::wstring err;
    if (!read_inputs(a, in, err)) {
        MessageBoxW(hwnd, err.c_str(), L"Input Error", MB_OK | MB_ICONWARNING);
        return;
    }
    in.payload_kg = 0.0;
    launch_task(hwnd, a, TaskKind::MaxPayload, in);
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

void draw_globe_panel(HDC hdc, const RECT& outer, const MissionResult& p, double view_lat_deg, double view_lon_deg) {
    draw_panel_frame(hdc, outer);

    if (p.globe_series.empty()) {
        const wchar_t* msg = L"3D globe view will appear after planning.";
        TextOutW(hdc, outer.left + 16, outer.top + 16, msg, lstrlenW(msg));
        return;
    }

    TextOutW(hdc, outer.left + 10, outer.top + 5, L"3D Earth View (Orthographic)", 28);
    TextOutW(hdc, outer.right - 270, outer.top + 5, L"Drag with left mouse button to rotate", 36);

    RECT in = outer;
    in.left += 14;
    in.right -= 14;
    in.top += 70;
    in.bottom -= 12;
    const int w = in.right - in.left;
    const int h = in.bottom - in.top;
    if (w <= 60 || h <= 60) return;

    const int cx = in.left + w / 2;
    const int cy = in.top + h / 2;
    const int radius_px = std::max(10, std::min(w, h) / 2 - 12);

    const Vec3 view_n = falcon9::normalize3(falcon9::ecef_from_geo(view_lat_deg, view_lon_deg, 0.0));
    const double view_lon = deg2rad(view_lon_deg);
    const double view_lat = deg2rad(view_lat_deg);
    const Vec3 view_u = falcon9::normalize3({-std::sin(view_lon), std::cos(view_lon), 0.0});
    const Vec3 view_v = falcon9::normalize3({
        -std::sin(view_lat) * std::cos(view_lon),
        -std::sin(view_lat) * std::sin(view_lon),
        std::cos(view_lat)});

    auto project = [&](const Vec3& q, POINT& out, double& depth) {
        depth = falcon9::dot3(q, view_n);
        out.x = cx + static_cast<int>(std::lround(falcon9::dot3(q, view_u) * radius_px));
        out.y = cy - static_cast<int>(std::lround(falcon9::dot3(q, view_v) * radius_px));
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
        TextOutW(hdc, pt.x + 6, pt.y - 8, tag, lstrlenW(tag));
    };

    draw_site(p.launch_lat_deg, p.launch_lon_deg, RGB(52, 73, 94), L"Launch");
    draw_site(p.ship_lat_deg, p.ship_lon_deg, RGB(22, 160, 133), L"Droneship");

    int ly = outer.top + 28;
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

    const Vec3 rhat = falcon9::normalize3(falcon9::ecef_from_geo(insertion_gp.lat_deg, insertion_gp.lon_deg, 0.0));
    const Vec3 east = falcon9::normalize3({-std::sin(deg2rad(insertion_gp.lon_deg)), std::cos(deg2rad(insertion_gp.lon_deg)), 0.0});
    const Vec3 north = falcon9::normalize3({
        -std::sin(deg2rad(insertion_gp.lat_deg)) * std::cos(deg2rad(insertion_gp.lon_deg)),
        -std::sin(deg2rad(insertion_gp.lat_deg)) * std::sin(deg2rad(insertion_gp.lon_deg)),
        std::cos(deg2rad(insertion_gp.lat_deg))});
    const double az_rad = deg2rad(out.orbit_target.launch_az_deg);
    const Vec3 horiz = falcon9::normalize3({
        north.x * std::cos(az_rad) + east.x * std::sin(az_rad),
        north.y * std::cos(az_rad) + east.y * std::sin(az_rad),
        north.z * std::cos(az_rad) + east.z * std::sin(az_rad),
    });
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
    constexpr int n_orbit = 540;
    post_orbit.pts.reserve(static_cast<size_t>(n_orbit + 1));
    for (int i = 0; i <= n_orbit; ++i) {
        const double nu = nu0 + (2.0 * 3.14159265358979323846 * static_cast<double>(i)) / static_cast<double>(n_orbit);
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
    ascent.name = L"Stage1 Ascent";
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

void draw_plot(HDC hdc, const RECT& outer, const MissionResult& p, double view_lat_deg, double view_lon_deg) {
    HBRUSH bg = CreateSolidBrush(RGB(245, 247, 250));
    FillRect(hdc, &outer, bg);
    DeleteObject(bg);

    RECT left{};
    RECT right{};
    split_plot_rects(outer, left, right);
    draw_profile_panel(hdc, left, p);
    draw_globe_panel(hdc, right, p, view_lat_deg, view_lon_deg);
}

void draw_sweep_charts(HDC hdc, const RECT& outer, const App& a) {
    HBRUSH bg = CreateSolidBrush(RGB(245, 247, 250));
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

LRESULT CALLBACK sweep_wndproc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_NCCREATE: {
            const CREATESTRUCTW* cs = reinterpret_cast<const CREATESTRUCTW*>(lp);
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(cs ? cs->lpCreateParams : nullptr));
            return TRUE;
        }
        case WM_PAINT: {
            App* a = app(hwnd);
            PAINTSTRUCT ps{};
            HDC hdc = BeginPaint(hwnd, &ps);
            RECT rc{};
            GetClientRect(hwnd, &rc);
            if (a) draw_sweep_charts(hdc, rc, *a);
            EndPaint(hwnd, &ps);
            return 0;
        }
        case WM_SIZE:
            InvalidateRect(hwnd, nullptr, TRUE);
            return 0;
        case WM_CLOSE:
            ShowWindow(hwnd, SW_HIDE);
            return 0;
        case WM_DESTROY: {
            App* a = app(hwnd);
            if (a && a->sweep_hwnd == hwnd) a->sweep_hwnd = nullptr;
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, 0);
            return 0;
        }
        default:
            return DefWindowProcW(hwnd, msg, wp, lp);
    }
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
            a->base_input = MissionRequest{};
            a->has_vehicle_config = g_startup_vehicle.has_config;
            a->vehicle_config_path = g_startup_vehicle.config_path;
            a->vehicle_config_error = g_startup_vehicle.error;
            if (g_startup_vehicle.has_config) a->base_input = g_startup_vehicle.input;
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
            a->btn_plan_coarse_1s = CreateWindowExW(0, L"BUTTON", L"Plan Coarse 1s Search", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 100, 30, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnPlanCoarse1s)), nullptr, nullptr);
            a->btn_plan_burnout_no_recovery = CreateWindowExW(0, L"BUTTON", L"Plan No-Recovery Burnout", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 100, 30, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnPlanBurnoutNoRecovery)), nullptr, nullptr);
            a->btn_max_payload = CreateWindowExW(0, L"BUTTON", L"Compute Max Payload", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 100, 30, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnMaxPayload)), nullptr, nullptr);
            a->btn_import_default_cfg = CreateWindowExW(0, L"BUTTON", L"Import Default Falcon9 Config", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 100, 30, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnImportDefaultCfg)), nullptr, nullptr);
            a->btn_prev_candidate = CreateWindowExW(0, L"BUTTON", L"<", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 48, 28, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnPrevCandidate)), nullptr, nullptr);
            a->btn_next_candidate = CreateWindowExW(0, L"BUTTON", L">", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 0, 0, 48, 28, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kBtnNextCandidate)), nullptr, nullptr);
            a->candidate_label = CreateWindowExW(0, L"STATIC", L"Candidates: none", WS_CHILD | WS_VISIBLE, 0, 0, 200, 24, hwnd, nullptr, nullptr, nullptr);
            a->list = CreateWindowExW(WS_EX_CLIENTEDGE, L"LISTBOX", L"", WS_CHILD | WS_VISIBLE | WS_VSCROLL | LBS_NOINTEGRALHEIGHT, 0, 0, 200, 100, hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kList)), nullptr, nullptr);
            set_font(a->btn, a->font);
            set_font(a->btn_plan_coarse_1s, a->font);
            set_font(a->btn_plan_burnout_no_recovery, a->font);
            set_font(a->btn_max_payload, a->font);
            set_font(a->btn_import_default_cfg, a->font);
            set_font(a->btn_prev_candidate, a->font);
            set_font(a->btn_next_candidate, a->font);
            set_font(a->candidate_label, a->font);
            set_font(a->list, a->font);
            sync_candidate_controls(*a);
            layout(hwnd, *a);
            a->sweep_hwnd = CreateWindowExW(
                0,
                kSweepWindowClass,
                L"Falcon9 Separation Sweep Charts",
                WS_OVERLAPPEDWINDOW,
                CW_USEDEFAULT,
                CW_USEDEFAULT,
                1360,
                420,
                hwnd,
                nullptr,
                GetModuleHandleW(nullptr),
                a);
            if (a->sweep_hwnd) ShowWindow(a->sweep_hwnd, SW_SHOWNOACTIVATE);
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
            if (a && LOWORD(wp) == kBtnPlanCoarse1s && HIWORD(wp) == BN_CLICKED) {
                run_plan_coarse_1s(hwnd, *a);
                return 0;
            }
            if (a && LOWORD(wp) == kBtnPlanBurnoutNoRecovery && HIWORD(wp) == BN_CLICKED) {
                run_plan_burnout_no_recovery(hwnd, *a);
                return 0;
            }
            if (a && LOWORD(wp) == kBtnPrevCandidate && HIWORD(wp) == BN_CLICKED) {
                if (a->selected_candidate_index > 0) --a->selected_candidate_index;
                sync_candidate_controls(*a);
                InvalidateRect(hwnd, &a->plot, TRUE);
                if (a->sweep_hwnd) InvalidateRect(a->sweep_hwnd, nullptr, TRUE);
                return 0;
            }
            if (a && LOWORD(wp) == kBtnNextCandidate && HIWORD(wp) == BN_CLICKED) {
                if (a->selected_candidate_index + 1 < a->plan.separation_candidates.size()) ++a->selected_candidate_index;
                sync_candidate_controls(*a);
                InvalidateRect(hwnd, &a->plot, TRUE);
                if (a->sweep_hwnd) InvalidateRect(a->sweep_hwnd, nullptr, TRUE);
                return 0;
            }
            if (a && LOWORD(wp) == kBtnMaxPayload && HIWORD(wp) == BN_CLICKED) {
                run_max_payload(hwnd, *a);
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
                const MissionResult display_plan = build_candidate_display_plan(*a);
                draw_plot(hdc, a->plot, display_plan, a->view_lat_deg, a->view_lon_deg);
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
                if (a->sweep_hwnd) {
                    DestroyWindow(a->sweep_hwnd);
                    a->sweep_hwnd = nullptr;
                }
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
                if (a->sweep_hwnd) {
                    DestroyWindow(a->sweep_hwnd);
                    a->sweep_hwnd = nullptr;
                }
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
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    if (!RegisterClassW(&wc)) return 1;

    WNDCLASSW sweep_wc{};
    sweep_wc.lpfnWndProc = sweep_wndproc;
    sweep_wc.hInstance = hi;
    sweep_wc.lpszClassName = kSweepWindowClass;
    sweep_wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    sweep_wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    if (!RegisterClassW(&sweep_wc)) return 1;

    HWND hwnd = CreateWindowExW(
        0,
        kMainWindowClass,
        L"Falcon9 GUI Planner (UPFG + Separation Search)",
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
