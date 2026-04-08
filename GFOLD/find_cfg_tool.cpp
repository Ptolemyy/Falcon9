#define NOMINMAX
#include "GFOLD_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <iostream>

#include <pybind11/embed.h>
#include <matplotlibcpp17/mplot3d.h>
#include <matplotlibcpp17/pyplot.h>

namespace py = pybind11;
namespace fs = std::filesystem;

static std::string trim_copy(const std::string& s) {
    size_t b = 0;
    size_t e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

static bool parse_pair_bracket(const std::string& v, double& a, double& b) {
    std::string t = trim_copy(v);
    if (t.size() < 5 || t.front() != '[' || t.back() != ']') return false;
    t = t.substr(1, t.size() - 2);
    const char sep = (t.find('|') != std::string::npos) ? '|' : ',';
    const size_t p = t.find(sep);
    if (p == std::string::npos) return false;
    try {
        a = std::stod(trim_copy(t.substr(0, p)));
        b = std::stod(trim_copy(t.substr(p + 1)));
    } catch (...) {
        return false;
    }
    return true;
}

static bool parse_triplet_bracket(const std::string& v, double& a, double& b, double& c) {
    std::string t = trim_copy(v);
    if (t.size() < 7 || t.front() != '[' || t.back() != ']') return false;
    t = t.substr(1, t.size() - 2);
    const char sep = (t.find('|') != std::string::npos) ? '|' : ',';
    const size_t p1 = t.find(sep);
    if (p1 == std::string::npos) return false;
    const size_t p2 = t.find(sep, p1 + 1);
    if (p2 == std::string::npos) return false;
    try {
        a = std::stod(trim_copy(t.substr(0, p1)));
        b = std::stod(trim_copy(t.substr(p1 + 1, p2 - p1 - 1)));
        c = std::stod(trim_copy(t.substr(p2 + 1)));
    } catch (...) {
        return false;
    }
    return true;
}

static bool is_supported_solver_n(int n) {
    return n == 3 || n == 10 || n == 25 || n == 50 || n == 100;
}

static bool apply_cfg_token(GFOLDConfig& cfg, const std::string& token) {
    const size_t eq = token.find('=');
    if (eq == std::string::npos) return true;
    const std::string key = trim_copy(token.substr(0, eq));
    const std::string val = trim_copy(token.substr(eq + 1));
    if (key.empty() || val.empty()) return true;

    try {
        if (key == "tf") cfg.tf = std::stod(val);
        else if (key == "g0") cfg.g0 = std::stod(val);
        else if (key == "Isp") cfg.Isp = std::stod(val);
        else if (key == "T_max") cfg.T_max = std::stod(val);
        else if (key == "m0") cfg.m0 = std::stod(val);
        else if (key == "glide") cfg.glide_slope_deg = std::stod(val);
        else if (key == "theta") cfg.max_angle_deg = std::stod(val);
        else if (key == "steps") {
            cfg.steps = std::stoi(val);
            cfg.solver_n = cfg.steps;
            if (!is_supported_solver_n(cfg.solver_n)) return false;
        }
        else if (key == "n" || key == "solver_n") {
            cfg.solver_n = std::stoi(val);
            cfg.steps = cfg.solver_n;
            if (!is_supported_solver_n(cfg.solver_n)) return false;
        }
        else if (key == "elapsed_time") cfg.elapsed_time = std::stod(val);
        else if (key == "throt") {
            double a = 0.0, b = 0.0;
            if (!parse_pair_bracket(val, a, b)) return false;
            cfg.throttle_min = a;
            cfg.throttle_max = b;
        } else if (key == "r0") {
            double a = 0.0, b = 0.0, c = 0.0;
            if (!parse_triplet_bracket(val, a, b, c)) return false;
            cfg.r0[0] = a;
            cfg.r0[1] = b;
            cfg.r0[2] = c;
        } else if (key == "v0") {
            double a = 0.0, b = 0.0, c = 0.0;
            if (!parse_triplet_bracket(val, a, b, c)) return false;
            cfg.v0[0] = a;
            cfg.v0[1] = b;
            cfg.v0[2] = c;
        }
    } catch (...) {
        return false;
    }
    return true;
}

static bool apply_cfg_string(GFOLDConfig& cfg, const std::string& cfg_text) {
    std::stringstream ss(cfg_text);
    std::string token;
    while (std::getline(ss, token, ';')) {
        token = trim_copy(token);
        if (token.empty()) continue;
        if (!apply_cfg_token(cfg, token)) return false;
    }
    return true;
}

static bool read_all_text(const std::string& path, std::string& out) {
    std::ifstream in(path, std::ios::in);
    if (!in) return false;
    std::ostringstream oss;
    oss << in.rdbuf();
    out = oss.str();
    return true;
}

static std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) out.push_back(item);
    return out;
}

static bool parse_mode1_sheet_index(const std::string& name, int& idx_out) {
    static const std::string kPrefix = "mode1_";
    if (name.rfind(kPrefix, 0) != 0) return false;
    const std::string tail = name.substr(kPrefix.size());
    if (tail.empty()) return false;
    for (char c : tail) {
        if (c < '0' || c > '9') return false;
    }
    try {
        idx_out = std::stoi(tail);
        return idx_out > 0;
    } catch (...) {
        return false;
    }
}

static bool parse_elapsed_input(std::string s, double& out_sec) {
    s = trim_copy(s);
    if (s.empty()) return false;
    if (s.back() == 's' || s.back() == 'S') s.pop_back();
    s = trim_copy(s);
    if (s.empty()) return false;
    try {
        out_sec = std::stod(s);
        return std::isfinite(out_sec);
    } catch (...) {
        return false;
    }
}

struct DebugModeInput {
    double elapsed_sec = std::numeric_limits<double>::quiet_NaN();
    bool direct_mode = false;
    int solver_n = -1; // optional override
    enum class SearchMode {
        Auto,
        Tf,
        Throt
    };
    SearchMode search_mode = SearchMode::Auto;
};

static const char* to_string(DebugModeInput::SearchMode mode) {
    switch (mode) {
        case DebugModeInput::SearchMode::Tf: return "tf";
        case DebugModeInput::SearchMode::Throt: return "throt";
        default: return "auto";
    }
}

static bool parse_solver_n_input(std::string s, int& out_n) {
    s = trim_copy(s);
    if (s.empty()) return false;

    for (char& c : s) {
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }

    if (s.rfind("solver_n", 0) == 0) {
        s = s.substr(8);
    } else if (!s.empty() && s.front() == 'n') {
        s = s.substr(1);
    }

    s = trim_copy(s);
    if (!s.empty() && s.front() == '=') s = s.substr(1);
    s = trim_copy(s);
    if (s.empty()) return false;

    try {
        const int n = std::stoi(s);
        if (!is_supported_solver_n(n)) return false;
        out_n = n;
        return true;
    } catch (...) {
        return false;
    }
}

static bool parse_yes_no_input(std::string s, bool& out) {
    s = trim_copy(s);
    if (s.empty()) return false;
    for (char& c : s) {
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    if (s == "y" || s == "yes" || s == "1" || s == "true") {
        out = true;
        return true;
    }
    if (s == "n" || s == "no" || s == "0" || s == "false") {
        out = false;
        return true;
    }
    return false;
}

static bool parse_search_mode_input(std::string s, DebugModeInput::SearchMode& out_mode) {
    s = trim_copy(s);
    if (s.empty()) return false;
    for (char& c : s) {
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    if (s == "tf") {
        out_mode = DebugModeInput::SearchMode::Tf;
        return true;
    }
    if (s == "throt" || s == "throttle" || s == "throt2" || s == "t2") {
        out_mode = DebugModeInput::SearchMode::Throt;
        return true;
    }
    if (s == "auto") {
        out_mode = DebugModeInput::SearchMode::Auto;
        return true;
    }
    return false;
}

static bool parse_debug_mode_input(std::string s, DebugModeInput& out) {
    s = trim_copy(s);
    if (s.empty()) return false;

    bool direct = false;
    if (!s.empty() && (s.front() == 'd' || s.front() == 'D')) {
        direct = true;
        s = s.substr(1);
    }
    s = trim_copy(s);
    if (s.empty()) return false;

    for (char& c : s) {
        if (c == ',' || c == ';') c = ' ';
    }

    std::stringstream ss(s);
    std::vector<std::string> parts;
    std::string part;
    while (ss >> part) parts.push_back(part);
    if (parts.empty() || parts.size() > 2) return false;

    int n_override = -1;
    double t = 0.0;
    if (!parse_elapsed_input(parts[0], t)) {
        // Also accept compact forms like d36n50 / 36n25.
        std::string compact = parts[0];
        for (char& c : compact) {
            if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
        }
        const size_t pos_n = compact.find('n', 1);
        if (pos_n == std::string::npos) return false;
        const std::string elapsed_part = parts[0].substr(0, pos_n);
        const std::string n_part = parts[0].substr(pos_n);
        if (!parse_elapsed_input(elapsed_part, t)) return false;
        if (!parse_solver_n_input(n_part, n_override)) return false;
    }

    if (parts.size() == 2) {
        int n2 = -1;
        if (!parse_solver_n_input(parts[1], n2)) return false;
        n_override = n2;
    }

    out.elapsed_sec = t;
    out.direct_mode = direct;
    out.solver_n = n_override;
    return true;
}

static bool find_latest_run_csv(const fs::path& dir, fs::path& out_csv) {
    std::error_code ec;
    if (!fs::exists(dir, ec)) return false;
    bool found = false;
    fs::file_time_type best_time{};
    fs::path best_path;
    for (const auto& e : fs::directory_iterator(dir, ec)) {
        if (ec) return false;
        if (!e.is_regular_file()) continue;
        if (e.path().extension() != ".csv") continue;
        const auto wt = e.last_write_time(ec);
        if (ec) continue;
        if (!found || wt > best_time) {
            found = true;
            best_time = wt;
            best_path = e.path();
        }
    }
    if (!found) return false;
    out_csv = best_path;
    return true;
}

static bool load_cfg_from_run_csv_by_time(
    const fs::path& run_csv_path,
    double target_elapsed_sec,
    std::string& selected_sheet_out,
    std::string& cfg_out) {
    selected_sheet_out.clear();
    cfg_out.clear();

    std::ifstream in(run_csv_path);
    if (!in) return false;

    std::string current_sheet;
    bool skip_header = false;
    std::vector<double> incoming_breaks;
    std::map<int, std::string> mode1_cfg;

    std::string line;
    while (std::getline(in, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();
        if (line.empty()) continue;

        if (line.rfind("sheet=", 0) == 0) {
            current_sheet = line.substr(6);
            skip_header = true;
            continue;
        }
        if (skip_header) {
            skip_header = false;
            continue;
        }
        if (current_sheet.empty()) continue;

        const auto tok = split_csv_line(line);
        if (tok.size() < 14) continue;

        // New CSV format inserts solver_n after index:
        // old : solve_seq,index,elapsed_time,...,cfg,note
        // new : solve_seq,index,solver_n,elapsed_time,...,cfg,note
        const int off = (tok.size() >= 15) ? 1 : 0;
        if (static_cast<int>(tok.size()) < (14 + off)) continue;
        const int elapsed_col = 2 + off;
        const int cfg_col = 12 + off;
        const int note_col = 13 + off;

        if (current_sheet == "receive") {
            if (tok[note_col] == "incoming_1") {
                try {
                    const double t = std::stod(tok[elapsed_col]);
                    if (std::isfinite(t)) {
                        if (incoming_breaks.empty() || std::fabs(incoming_breaks.back() - t) > 1e-9) {
                            incoming_breaks.push_back(t);
                        }
                    }
                } catch (...) {
                }
            }
            continue;
        }

        int mode_idx = 0;
        if (!parse_mode1_sheet_index(current_sheet, mode_idx)) continue;
        if (mode1_cfg.find(mode_idx) != mode1_cfg.end()) continue;
        if (!tok[cfg_col].empty()) mode1_cfg[mode_idx] = tok[cfg_col];
    }

    if (incoming_breaks.empty() || mode1_cfg.empty()) return false;

    int best_break_idx = 0;
    double best_abs_dt = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(incoming_breaks.size()); ++i) {
        const double dt = std::fabs(incoming_breaks[i] - target_elapsed_sec);
        if (dt < best_abs_dt) {
            best_abs_dt = dt;
            best_break_idx = i;
        }
    }

    const int expected_mode1_idx = best_break_idx + 1;
    auto it = mode1_cfg.find(expected_mode1_idx);
    if (it == mode1_cfg.end()) {
        int best_mode_idx = -1;
        int best_dist = std::numeric_limits<int>::max();
        for (const auto& kv : mode1_cfg) {
            const int d = std::abs(kv.first - expected_mode1_idx);
            if (d < best_dist) {
                best_dist = d;
                best_mode_idx = kv.first;
            }
        }
        if (best_mode_idx < 0) return false;
        it = mode1_cfg.find(best_mode_idx);
    }

    if (it == mode1_cfg.end() || it->second.empty()) return false;
    selected_sheet_out = "mode1_" + std::to_string(it->first);
    cfg_out = it->second;
    return true;
}

// Evaluate m(tf). If infeasible, return nullopt.
static std::optional<double> eval_m(GFOLDSolver& solver, GFOLDConfig& cfg, double tf)
{
    cfg.tf = tf;
    solver.set_config(cfg);
    if (!solver.solve()) {
        return std::nullopt;
    }
    const double m = solver.terminal_mass();
    if (m == m) return m;
    return std::nullopt;
}

// Coarse scan to find feasible bracket and best m(tf) sample.
struct Bracket {
    double a;
    double b;
    double best_tf;
    double best_m;
    std::vector<double> tfs;
    std::vector<double> ms;
};

static Bracket coarse_bracket(GFOLDSolver& solver, GFOLDConfig& cfg,
                              double tf_min, double tf_max, double tf_step)
{
    Bracket out;
    out.a = tf_min;
    out.b = tf_max;
    out.best_tf = tf_min;
    out.best_m = -std::numeric_limits<double>::infinity();

    bool seen_feasible = false;
    double first_feas = 0.0;
    double last_feas = 0.0;

    for (double tf = tf_min; tf <= tf_max + 1e-12; tf += tf_step) {
        auto mopt = eval_m(solver, cfg, tf);
        if (!mopt.has_value()) {
            std::cout << "tf = " << tf << " infeasible, status = "
                      << solver.status() << "\n";
            continue;
        }
        double m = *mopt;

        out.tfs.push_back(tf);
        out.ms.push_back(m);

        if (!seen_feasible) {
            seen_feasible = true;
            first_feas = tf;
        }
        last_feas = tf;

        if (m > out.best_m) {
            out.best_m = m;
            out.best_tf = tf;
        }

        std::cout << "tf = " << tf << ", m = " << m
                  << " status = " << solver.status() << "\n";
    }

    if (!seen_feasible) {
        out.a = tf_min;
        out.b = tf_min;
        out.best_tf = tf_min;
        out.best_m = -std::numeric_limits<double>::infinity();
        return out;
    }

    out.a = first_feas;
    out.b = last_feas;
    return out;
}

struct ThrottleSearchResult {
    double best_throttle_max = 1.0;
    long long solve_calls = 0;
    bool feasible = false;
    std::vector<double> throttle_max_samples;
    std::vector<double> mass_samples;
};

static ThrottleSearchResult search_min_feasible_throttle_max(
    const GFOLDConfig& cfg_in,
    double throttle_lo,
    double throttle_hi = 1.0,
    int iters = 20) {
    ThrottleSearchResult res;
    constexpr double eps = 1e-9;

    double lo = std::max(0.0, throttle_lo);
    double hi = std::min(1.0, throttle_hi);
    if (hi <= lo + eps) return res;

    GFOLDSolver solver(cfg_in);
    auto eval_feasible = [&](double throttle_max) -> bool {
        res.solve_calls++;
        GFOLDConfig trial = cfg_in;
        trial.throttle_max = throttle_max;
        solver.set_config(trial);
        if (!solver.solve()) return false;

        const double m = solver.terminal_mass();
        if (!std::isfinite(m)) return false;
        res.throttle_max_samples.push_back(throttle_max);
        res.mass_samples.push_back(m);
        return true;
    };

    // Dense area scan on full range, then refine around the left-most feasible point.
    const int coarse_points = std::max(160, iters * 12); // iters=20 -> 240 points
    const double coarse_step = (hi - lo) / static_cast<double>(coarse_points);

    bool has_feasible = false;
    double best_throt = hi;
    for (int i = 0; i <= coarse_points; ++i) {
        const double t = (i == coarse_points) ? hi : (lo + coarse_step * static_cast<double>(i));
        if (eval_feasible(t)) {
            if (!has_feasible || t < best_throt) {
                has_feasible = true;
                best_throt = t;
            }
        }
    }

    if (!has_feasible) return res;

    // Local refinement near the best coarse candidate.
    const int refine_points = std::max(100, iters * 6); // iters=20 -> 120 points
    const double refine_lo = std::max(lo, best_throt - coarse_step);
    const double refine_hi = best_throt;
    if (refine_hi > refine_lo + eps) {
        const double refine_step = (refine_hi - refine_lo) / static_cast<double>(refine_points);
        for (int i = 0; i <= refine_points; ++i) {
            const double t = (i == refine_points)
                                 ? refine_hi
                                 : (refine_lo + refine_step * static_cast<double>(i));
            if (eval_feasible(t) && t < best_throt) best_throt = t;
        }
    }

    res.feasible = true;
    res.best_throttle_max = best_throt;
    return res;
}

// Golden-section maximize inside feasible bracket.
static std::pair<double,double> golden_maximize(
    GFOLDSolver& solver, GFOLDConfig& cfg,
    double a, double b,
    int max_iter = 60,
    double tol = 1e-4)
{
    const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    const double invphi = 1.0 / phi;

    double c = b - (b - a) * invphi;
    double d = a + (b - a) * invphi;

    auto fc_opt = eval_m(solver, cfg, c);
    auto fd_opt = eval_m(solver, cfg, d);

    auto fval = [](const std::optional<double>& x) {
        return x.has_value() ? *x : -std::numeric_limits<double>::infinity();
    };

    double fc = fval(fc_opt);
    double fd = fval(fd_opt);

    for (int it = 0; it < max_iter && (b - a) > tol; ++it) {
        if (fc < fd) {
            a = c;
            c = d;
            fc = fd;

            d = a + (b - a) * invphi;
            fd_opt = eval_m(solver, cfg, d);
            fd = fval(fd_opt);
        } else {
            b = d;
            d = c;
            fd = fc;

            c = b - (b - a) * invphi;
            fc_opt = eval_m(solver, cfg, c);
            fc = fval(fc_opt);
        }
    }

    std::vector<double> cand = {a, b, c, d};
    double best_tf = cand[0];
    double best_m  = -std::numeric_limits<double>::infinity();

    for (double tf : cand) {
        auto mopt = eval_m(solver, cfg, tf);
        double m = mopt.has_value() ? *mopt : -std::numeric_limits<double>::infinity();
        if (m > best_m) {
            best_m = m;
            best_tf = tf;
        }
    }
    return {best_tf, best_m};
}

int main(int argc, char** argv)
{
    // Problem setup: copy from main.cpp
    GFOLDConfig cfg;
    cfg.steps = 100;
    cfg.tf = 4.0053921228;
    cfg.g0 = 9.8100000000;
    cfg.Isp = 252.6960000000;
    cfg.T_max = 169780.0000000000;
    cfg.throttle_min = 0.3000000000;
    cfg.throttle_max = 0.8000000000;
    cfg.m0 = 5545.0000000000;
    cfg.r0[0] = 113.1000000000;
    cfg.r0[1] = 0.1000000000;
    cfg.r0[2] = 12.8000000000;
    cfg.v0[0] = -48.0600000000;
    cfg.v0[1] = 0.8600000000;
    cfg.v0[2] = -9.2200000000;
    cfg.glide_slope_deg = 30.000000;
    cfg.max_angle_deg = 20.000000;
    cfg.elapsed_time = 36.0000000000;
    cfg.solver_n = cfg.steps;
    bool has_user_center = false;
    double user_center = std::numeric_limits<double>::quiet_NaN();
    bool direct_mode = false;
    DebugModeInput::SearchMode search_mode = DebugModeInput::SearchMode::Auto;
    std::string selected_sheet_name;

    // Optional override:
    //   find_cfg "<cfg_text>"
    //   find_cfg --cfg-file <path>
    // cfg_text supports:
    //   n=<3|10|25|50|100>
    //   solver_n=<3|10|25|50|100>
    //   steps=<3|10|25|50|100> (kept for compatibility, also sets solver_n)
    // No args:
    //   find latest run csv in current directory, ask elapsed time, and map to mode1 cfg.
    if (argc >= 2) {
        std::string cfg_text;
        if (std::string(argv[1]) == "--cfg-file") {
            if (argc < 3 || !read_all_text(argv[2], cfg_text)) {
                std::cerr << "failed to read cfg file\n";
                return 1;
            }
        } else {
            cfg_text = argv[1];
        }
        if (!cfg_text.empty()) {
            if (!apply_cfg_string(cfg, cfg_text)) {
                std::cerr << "failed to parse cfg text\n";
                return 1;
            }
        }
    } else {
        fs::path latest_csv;
        if (find_latest_run_csv(fs::current_path(), latest_csv)) {
            std::cout << "[debug] latest_csv=" << latest_csv.string() << "\n";
            std::cout << "[debug] input elapsed time (e.g. 36), empty to skip: ";
            std::string input_elapsed;
            if (std::getline(std::cin, input_elapsed)) {
                input_elapsed = trim_copy(input_elapsed);
                if (!input_elapsed.empty()) {
                    double elapsed_sec = std::numeric_limits<double>::quiet_NaN();
                    if (!parse_elapsed_input(input_elapsed, elapsed_sec)) {
                        std::cout << "[debug] invalid_time=" << input_elapsed << "\n";
                        return 1;
                    }

                    bool user_direct = false;
                    std::cout << "[debug] direct mode? (y/n, default n): ";
                    std::string input_direct;
                    if (std::getline(std::cin, input_direct)) {
                        input_direct = trim_copy(input_direct);
                        if (!input_direct.empty()) {
                            if (!parse_yes_no_input(input_direct, user_direct)) {
                                std::cout << "[debug] invalid_direct=" << input_direct << "\n";
                                return 1;
                            }
                        }
                    }

                    int user_solver_n = -1;
                    std::cout << "[debug] solver_n override? (10/25/50/100, empty keep): ";
                    std::string input_n;
                    if (std::getline(std::cin, input_n)) {
                        input_n = trim_copy(input_n);
                        if (!input_n.empty()) {
                            if (!parse_solver_n_input(input_n, user_solver_n)) {
                                std::cout << "[debug] invalid_solver_n=" << input_n << "\n";
                                return 1;
                            }
                        }
                    }

                    DebugModeInput::SearchMode user_search_mode = DebugModeInput::SearchMode::Auto;
                    std::cout << "[debug] search mode? (tf/throt/auto, default auto): ";
                    std::string input_search_mode;
                    if (std::getline(std::cin, input_search_mode)) {
                        input_search_mode = trim_copy(input_search_mode);
                        if (!input_search_mode.empty()) {
                            if (!parse_search_mode_input(input_search_mode, user_search_mode)) {
                                std::cout << "[debug] invalid_search_mode=" << input_search_mode << "\n";
                                return 1;
                            }
                        }
                    }

                    has_user_center = true;
                    user_center = elapsed_sec;
                    direct_mode = user_direct;
                    search_mode = user_search_mode;

                    std::string cfg_text;
                    if (load_cfg_from_run_csv_by_time(latest_csv, user_center, selected_sheet_name, cfg_text)) {
                        std::cout << (direct_mode ? "[direct] " : "[debug] ")
                                  << "selected_sheet=" << selected_sheet_name << "\n";
                        std::cout << (direct_mode ? "[direct] " : "[debug] ")
                                  << "cfg=" << cfg_text << "\n";
                        std::cout << (direct_mode ? "[direct] " : "[debug] ")
                                  << "search_mode=" << to_string(search_mode) << "\n";
                        if (!apply_cfg_string(cfg, cfg_text)) {
                            std::cerr << "failed to parse cfg from latest csv\n";
                            return 1;
                        }
                        if (user_solver_n > 0) {
                            cfg.solver_n = user_solver_n;
                            cfg.steps = user_solver_n;
                        }
                    } else {
                        std::cout << "[debug] lookup_failed t=" << user_center << "s\n";
                        return 1;
                    }
                }
            }
        } else {
            std::cout << "[debug] latest_csv_not_found_in=" << fs::current_path().string() << "\n";
            return 1;
        }
    }

    GFOLDSolver solver(cfg);

    if (direct_mode) {
        if (!std::isfinite(cfg.tf) || cfg.tf <= 0.0) {
            std::cerr << "[direct] invalid cfg.tf=" << cfg.tf << "\n";
            return 1;
        }
        solver.set_config(cfg);
        bool ok = solver.solve();
        std::cout << "[direct] n=" << cfg.solver_n
                  << " tf=" << cfg.tf
                  << " status=" << solver.status() << "\n";
        if (search_mode == DebugModeInput::SearchMode::Throt) {
            auto throt_retry = search_min_feasible_throttle_max(cfg, cfg.throttle_max, 1.0, 20);
            if (throt_retry.feasible) {
                cfg.throttle_max = throt_retry.best_throttle_max;
                std::cout << "[direct] throt_search ok throt2=" << cfg.throttle_max
                          << " calls=" << throt_retry.solve_calls << "\n";
                solver.set_config(cfg);
                ok = solver.solve();
                std::cout << "[direct] retry status=" << solver.status() << "\n";
            } else {
                std::cout << "[direct] throt_search infeasible calls="
                          << throt_retry.solve_calls << " (keep current solve)\n";
            }
            if (!ok) return 0;
        } else if (!ok) {
            std::cout << "[direct] infeasible at tf=" << cfg.tf << "\n";
            const bool use_throt_search =
                (search_mode == DebugModeInput::SearchMode::Throt) ||
                (search_mode == DebugModeInput::SearchMode::Auto && cfg.tf < 5.0);
            if (use_throt_search) {
                auto throt_retry = search_min_feasible_throttle_max(cfg, cfg.throttle_max, 1.0, 20);
                if (throt_retry.feasible) {
                    cfg.throttle_max = throt_retry.best_throttle_max;
                    std::cout << "[direct] throt_search ok throt2=" << cfg.throttle_max
                              << " calls=" << throt_retry.solve_calls << "\n";
                    solver.set_config(cfg);
                    ok = solver.solve();
                    std::cout << "[direct] retry status=" << solver.status() << "\n";
                } else {
                    std::cout << "[direct] throt_search infeasible calls="
                              << throt_retry.solve_calls << "\n";
                }
            }
            if (!ok) return 0;
        }
        const double direct_m = solver.terminal_mass();
        std::cout << "[direct] terminal_m=" << direct_m << "\n";

        GFOLDThrustProfile prof = solver.compute_thrust_profile();
        GFOLDSolution sol = solver.solution();

        py::scoped_interpreter guard{};
        auto plt = matplotlibcpp17::pyplot::import();

        auto fig = plt.figure(
            Args(),
            Kwargs("figsize"_a = py::make_tuple(10, 6))
        );

        auto ax4 = fig.add_subplot(Args(2, 1, 1));
        ax4.plot(Args(prof.t, prof.thrust_N), Kwargs("color"_a = "blue"));

        const double y1 = cfg.T_max * cfg.throttle_max;
        const double y2 = cfg.T_max * cfg.throttle_min;
        ax4.plot(Args(prof.t, std::vector<double>(prof.t.size(), y1)),
                 Kwargs("linestyle"_a="--", "color"_a="red"));
        ax4.plot(Args(prof.t, std::vector<double>(prof.t.size(), y2)),
                 Kwargs("linestyle"_a="--", "color"_a="red"));
        ax4.set_title(Args("Thrust (N)"));

        auto ax5 = fig.add_subplot(Args(2, 1, 2));
        ax5.plot(Args(prof.t, std::vector<double>(prof.t.size(), cfg.max_angle_deg)),
                 Kwargs("linestyle"_a="--", "color"_a="red"));
        ax5.plot(Args(prof.t, prof.angle_deg), Kwargs("color"_a="blue"));
        ax5.set_title(Args("Thrust angle (deg)"));

        if (sol.steps > 0 &&
            static_cast<int>(sol.rx.size()) == sol.steps &&
            static_cast<int>(sol.ry.size()) == sol.steps &&
            static_cast<int>(sol.rz.size()) == sol.steps) {
            auto fig_r = plt.figure(
                Args(),
                Kwargs("figsize"_a = py::make_tuple(8, 6))
            );
            auto axr = fig_r.add_subplot(Args(1, 1, 1), Kwargs("projection"_a = "3d"));
            axr.plot(Args(sol.rx, sol.ry, sol.rz),
                     Kwargs("color"_a = "tab:blue", "linewidth"_a = 2.0));
            axr.set_title(Args("R 3D Trajectory (Direct Mode)"));
            axr.set_xlabel(Args("Up (x)"));
            axr.set_ylabel(Args("North (y)"));
            axr.set_zlabel(Args("East (z)"));
            axr.grid(Args(true));
        } else {
            std::cout << "Skip R 3D plot: invalid r trajectory.\n";
        }

        plt.show();
        return 0;
    }

    Bracket br;
    double best_tf = cfg.tf;
    double best_m = -std::numeric_limits<double>::infinity();
    std::vector<double> metric_x;
    std::vector<double> metric_y;
    std::string metric_x_label = "tf";
    std::string metric_y_label = "m(tf)";
    std::string metric_title = "m(tf) (feasible points only)";

    const bool throt_only_mode = (search_mode == DebugModeInput::SearchMode::Throt);
    if (throt_only_mode) {
        std::cout << "[search] mode=throt n=" << cfg.solver_n
                  << " tf_fixed=" << cfg.tf
                  << " throt2_range=(" << cfg.throttle_max << ",1]\n";
        auto throt_retry = search_min_feasible_throttle_max(cfg, cfg.throttle_max, 1.0, 20);
        if (!throt_retry.feasible) {
            std::cout << "No feasible throt2 in (" << cfg.throttle_max << ",1], calls="
                      << throt_retry.solve_calls << "\n";
            return 0;
        }
        cfg.throttle_max = throt_retry.best_throttle_max;
        std::cout << "throt best = " << cfg.throttle_max
                  << ", calls = " << throt_retry.solve_calls << "\n";
        solver.set_config(cfg);
        if (!solver.solve()) {
            std::cout << "throt best solution infeasible, status = "
                      << solver.status() << '\n';
            return 0;
        }
        best_tf = cfg.tf;
        best_m = solver.terminal_mass();
        br.a = cfg.tf;
        br.b = cfg.tf;
        br.best_tf = cfg.tf;
        br.best_m = best_m;
        br.tfs.push_back(cfg.tf);
        br.ms.push_back(best_m);

        metric_x = throt_retry.throttle_max_samples;
        metric_y = throt_retry.mass_samples;
        if (metric_x.size() != metric_y.size()) {
            metric_x.clear();
            metric_y.clear();
        }
        if (metric_x.empty()) {
            metric_x.push_back(cfg.throttle_max);
            metric_y.push_back(best_m);
        }
        std::vector<std::pair<double, double>> pairs;
        pairs.reserve(metric_x.size());
        for (size_t i = 0; i < metric_x.size(); ++i) {
            pairs.emplace_back(metric_x[i], metric_y[i]);
        }
        std::sort(pairs.begin(), pairs.end(),
                  [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
        metric_x.clear();
        metric_y.clear();
        metric_x.reserve(pairs.size());
        metric_y.reserve(pairs.size());
        for (const auto& p : pairs) {
            metric_x.push_back(p.first);
            metric_y.push_back(p.second);
        }
        metric_x_label = "throt2";
        metric_y_label = "m(throt2)";
        metric_title = "m(throt2) (feasible points only)";
    } else {
        double tf_min = 1.0;
        double tf_max = 6.0;
        if (has_user_center && std::isfinite(user_center)) {
            tf_min = user_center - 5.0;
            tf_max = user_center + 5.0;
            if (tf_min < 0.1) tf_min = 0.1;
            if (tf_max <= tf_min + 1e-6) tf_max = tf_min + 10.0;
        }
        const double tf_step = 0.1;
        std::cout << "[search] n=" << cfg.solver_n
                  << " tf_range=[" << tf_min << "," << tf_max << "]\n";

        br = coarse_bracket(solver, cfg, tf_min, tf_max, tf_step);
        if (br.ms.empty()) {
            std::cout << "No feasible tf in [" << tf_min << "," << tf_max << "].\n";
            return 0;
        }

        std::cout << "coarse best tf = " << br.best_tf << ", m = " << br.best_m
                  << " (feasible range approx [" << br.a << ", " << br.b << "])\n";

        auto best_pair = golden_maximize(solver, cfg, br.a, br.b, /*max_iter=*/80, /*tol=*/1e-4);
        best_tf = best_pair.first;
        best_m = best_pair.second;
        std::cout << "golden best tf = " << best_tf << ", m = " << best_m << "\n";

        metric_x = br.tfs;
        metric_y = br.ms;
    }

    cfg.tf = best_tf;
    solver.set_config(cfg);
    bool ok = solver.solve();
    const bool use_throt_search_after_tf =
        (search_mode == DebugModeInput::SearchMode::Throt) ||
        (search_mode == DebugModeInput::SearchMode::Auto && cfg.tf < 5.0);
    if (!ok && use_throt_search_after_tf) {
        auto throt_retry = search_min_feasible_throttle_max(cfg, cfg.throttle_max, 1.0, 20);
        if (throt_retry.feasible) {
            cfg.throttle_max = throt_retry.best_throttle_max;
            std::cout << "[search] throt_search ok throt2=" << cfg.throttle_max
                      << " calls=" << throt_retry.solve_calls << "\n";
            solver.set_config(cfg);
            ok = solver.solve();
            std::cout << "[search] retry status=" << solver.status() << "\n";
        } else {
            std::cout << "[search] throt_search infeasible calls="
                      << throt_retry.solve_calls << "\n";
        }
    }
    if (!ok) {
        std::cout << "Best tf solution infeasible, status = "
                  << solver.status() << '\n';
        return 0;
    }

    GFOLDThrustProfile prof = solver.compute_thrust_profile();
    GFOLDSolution sol = solver.solution();

    py::scoped_interpreter guard{};
    auto plt = matplotlibcpp17::pyplot::import();

    auto fig = plt.figure(
        Args(),
        Kwargs("figsize"_a = py::make_tuple(10, 8))
    );

    auto ax1 = fig.add_subplot(Args(6, 1, 1));
    ax1.plot(Args(metric_x, metric_y), Kwargs("color"_a = "blue", "linewidth"_a = 2.0));
    ax1.set_xlabel(Args(metric_x_label));
    ax1.set_ylabel(Args(metric_y_label));
    ax1.set_title(Args(metric_title));
    ax1.grid(Args(true));

    auto ax4 = fig.add_subplot(Args(6, 1, 4));
    ax4.plot(Args(prof.t, prof.thrust_N), Kwargs("color"_a = "blue"));

    double y1 = cfg.T_max * cfg.throttle_max;
    double y2 = cfg.T_max * cfg.throttle_min;

    ax4.plot(Args(prof.t, std::vector<double>(prof.t.size(), y1)),
             Kwargs("linestyle"_a="--", "color"_a="red"));
    ax4.plot(Args(prof.t, std::vector<double>(prof.t.size(), y2)),
             Kwargs("linestyle"_a="--", "color"_a="red"));
    ax4.set_title(Args("Thrust (N)"));

    auto ax5 = fig.add_subplot(Args(6, 1, 5));
    ax5.plot(Args(prof.t, std::vector<double>(prof.t.size(), cfg.max_angle_deg)),
             Kwargs("linestyle"_a="--", "color"_a="red"));
    ax5.plot(Args(prof.t, prof.angle_deg), Kwargs("color"_a="blue"));
    ax5.set_title(Args("Thrust angle (deg)"));

    if (sol.steps > 0 &&
        static_cast<int>(sol.rx.size()) == sol.steps &&
        static_cast<int>(sol.ry.size()) == sol.steps &&
        static_cast<int>(sol.rz.size()) == sol.steps) {
        auto fig_r = plt.figure(
            Args(),
            Kwargs("figsize"_a = py::make_tuple(8, 6))
        );
        auto axr = fig_r.add_subplot(Args(1, 1, 1), Kwargs("projection"_a = "3d"));
        axr.plot(Args(sol.rx, sol.ry, sol.rz),
                 Kwargs("color"_a = "tab:blue", "linewidth"_a = 2.0));
        axr.set_title(Args("R 3D Trajectory"));
        axr.set_xlabel(Args("Up (x)"));
        axr.set_ylabel(Args("North (y)"));
        axr.set_zlabel(Args("East (z)"));
        axr.grid(Args(true));
    } else {
        std::cout << "Skip R 3D plot: invalid r trajectory.\n";
    }

    plt.show();
    return 0;
}
