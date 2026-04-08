#include "GFOLD_solver.hpp"
#include "find_cfg.hpp"

#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cctype>
#include <array>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <ctime>
#include <vector>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

bool plot_from_run_csv(const std::string& run_csv_path);

static std::string read_last_line(const fs::path& p) {
    std::ifstream in(p);
    if (!in) return "";
    std::string line, last;
    while (std::getline(in, line)) {
        if (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
            while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();
        }
        if (!line.empty()) last.swap(line);
    }
    return last;
}

static std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) out.push_back(item);
    return out;
}

// Read the oldest line from send.txt and remove it from the file.
static std::string consume_first_line(const fs::path& p) {
    std::ifstream in(p);
    if (!in) return "";
    std::vector<std::string> lines;
    std::string l;
    while (std::getline(in, l)) {
        while (!l.empty() && (l.back() == '\n' || l.back() == '\r')) l.pop_back();
        if (!l.empty()) lines.push_back(l);
    }
    if (lines.empty()) return "";
    std::string first = lines.front();
    lines.erase(lines.begin());

    std::ofstream out(p, std::ios::trunc);
    for (size_t i = 0; i < lines.size(); ++i) {
        out << lines[i];
        if (i + 1 < lines.size()) out << "\n";
    }
    return first;
}

static bool atomic_write(const fs::path& p, const std::string& content) {
    fs::path tmp = p;
    tmp += ".tmp";

    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << content;
    out.close();

    std::error_code ec;
    fs::remove(p, ec);
    fs::rename(tmp, p, ec);
    return !ec;
}

static void write_compute_finish(const fs::path& recv_path, int status_code) {
    std::ostringstream oss;
    oss << "COMPUTE_FINISH," << status_code << "\n";
    if (!atomic_write(recv_path, oss.str())) {
//        std::cerr << "Failed to write receive.txt\n";
    }
}

static void write_request_mode0(const fs::path& recv_path, const std::string& reason) {
    std::ostringstream oss;
    oss << "REQUEST_MODE0,1";
    if (!reason.empty()) oss << "," << reason;
    oss << "\n";
    if (!atomic_write(recv_path, oss.str())) {
//        std::cerr << "Failed to write receive.txt\n";
    }
}

enum class IncomingType {
    Mode0,
    Mode1,
    Info,
    End,
    Unknown
};

static std::string to_upper_ascii(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return s;
}

static IncomingType parse_incoming_type(const std::string& token0) {
    if (token0 == "0") return IncomingType::Mode0;
    if (token0 == "1") return IncomingType::Mode1;
    const std::string u = to_upper_ascii(token0);
    if (u == "INFO") return IncomingType::Info;
    if (u == "END") return IncomingType::End;
    return IncomingType::Unknown;
}

struct InfoEndRvm {
    std::string tag;
    double r0 = std::numeric_limits<double>::quiet_NaN();
    double r1 = std::numeric_limits<double>::quiet_NaN();
    double r2 = std::numeric_limits<double>::quiet_NaN();
    double v0 = std::numeric_limits<double>::quiet_NaN();
    double v1 = std::numeric_limits<double>::quiet_NaN();
    double v2 = std::numeric_limits<double>::quiet_NaN();
    double m = std::numeric_limits<double>::quiet_NaN();
};

static bool parse_info_end_rvm(const std::vector<std::string>& tok, InfoEndRvm& out) {
    out.tag = tok.empty() ? "" : tok[0];
    bool ok = true;
    auto parse_one = [&](size_t idx, double& dst) {
        if (idx >= tok.size()) {
            ok = false;
            return;
        }
        try {
            dst = std::stod(tok[idx]);
        } catch (...) {
            ok = false;
        }
    };

    parse_one(1, out.r0);
    parse_one(2, out.r1);
    parse_one(3, out.r2);
    parse_one(4, out.v0);
    parse_one(5, out.v1);
    parse_one(6, out.v2);
    parse_one(10, out.m);
    return ok;
}

static bool try_parse_double_at(const std::vector<std::string>& tok, size_t idx, double& out) {
    if (idx >= tok.size()) return false;
    try {
        out = std::stod(tok[idx]);
        return true;
    } catch (...) {
        return false;
    }
}

static bool try_parse_r_xyz(const std::vector<std::string>& tok, std::array<double, 3>& r_xyz) {
    if (tok.size() < 4) return false;
    try {
        // send format is: mode, up, north, east, ...
        r_xyz[0] = std::stod(tok[1]); // up
        r_xyz[1] = std::stod(tok[2]); // north
        r_xyz[2] = std::stod(tok[3]); // east
    } catch (...) {
        return false;
    }
    return true;
}

static bool try_parse_v_enu_t(const std::vector<std::string>& tok, std::array<double, 4>& v_enu_t) {
    // send format (full):   mode,up,north,east,vu,vn,ve,speed,Tmax,Isp,m,th1,th2,theta,ygs,elapsed
    // send format (compact):mode,up,north,east,vu,vn,ve,speed,Tmax,Isp,m,elapsed
    if (tok.size() < 12) return false;
    size_t t_idx = std::numeric_limits<size_t>::max();
    if (tok.size() > 15) t_idx = 15;
    else if (tok.size() > 11) t_idx = 11;
    else return false;
    try {
        v_enu_t[0] = std::stod(tok[t_idx]); // t
        v_enu_t[1] = std::stod(tok[4]);  // v_up
        v_enu_t[2] = std::stod(tok[5]);  // v_north
        v_enu_t[3] = std::stod(tok[6]);  // v_east
    } catch (...) {
        return false;
    }
    return true;
}

static bool write_xyz_csv(const fs::path& out_path,
                          const std::vector<double>& x,
                          const std::vector<double>& y,
                          const std::vector<double>& z) {
    if (x.size() != y.size() || x.size() != z.size()) return false;
    std::ostringstream oss;
    oss << std::setprecision(10) << std::fixed;
    for (size_t i = 0; i < x.size(); ++i) {
        oss << x[i] << "," << y[i] << "," << z[i] << "\n";
    }
    return atomic_write(out_path, oss.str());
}

static bool write_xyz_csv(const fs::path& out_path, const std::vector<std::array<double, 3>>& pts) {
    std::ostringstream oss;
    oss << std::setprecision(10) << std::fixed;
    for (const auto& p : pts) {
        oss << p[0] << "," << p[1] << "," << p[2] << "\n";
    }
    return atomic_write(out_path, oss.str());
}

static bool write_tuvw_csv(const fs::path& out_path, const std::vector<std::array<double, 4>>& pts) {
    std::ostringstream oss;
    oss << std::setprecision(10) << std::fixed;
    for (const auto& p : pts) {
        oss << p[0] << "," << p[1] << "," << p[2] << "," << p[3] << "\n";
    }
    return atomic_write(out_path, oss.str());
}

static bool write_tuvw_csv(const fs::path& out_path,
                           const std::vector<double>& t,
                           const std::vector<double>& u,
                           const std::vector<double>& v,
                           const std::vector<double>& w) {
    if (t.size() != u.size() || t.size() != v.size() || t.size() != w.size()) return false;
    std::ostringstream oss;
    oss << std::setprecision(10) << std::fixed;
    for (size_t i = 0; i < t.size(); ++i) {
        oss << t[i] << "," << u[i] << "," << v[i] << "," << w[i] << "\n";
    }
    return atomic_write(out_path, oss.str());
}

struct CsvLogRow {
    std::string sheet = "mode0";
    long long solve_seq = 0;
    int index = 0;
    int solver_n = -1;
    double elapsed_time = std::numeric_limits<double>::quiet_NaN();
    double ux = std::numeric_limits<double>::quiet_NaN();
    double uy = std::numeric_limits<double>::quiet_NaN();
    double uz = std::numeric_limits<double>::quiet_NaN();
    double rx = std::numeric_limits<double>::quiet_NaN();
    double ry = std::numeric_limits<double>::quiet_NaN();
    double rz = std::numeric_limits<double>::quiet_NaN();
    double vx = std::numeric_limits<double>::quiet_NaN();
    double vy = std::numeric_limits<double>::quiet_NaN();
    double vz = std::numeric_limits<double>::quiet_NaN();
    std::string cfg;
    std::string note;
};

enum class CsvLogTaskType {
    AppendRows,
    FlushToFile,
    ResetRun,
    Stop
};

struct CsvLogTask {
    CsvLogTaskType type = CsvLogTaskType::AppendRows;
    std::vector<CsvLogRow> rows;
    std::string filename;
    std::shared_ptr<std::promise<bool>> done;
};

static void write_csv_value(std::ostringstream& oss, double v) {
    if (std::isfinite(v)) oss << v;
}

static std::string format_solver_cfg(const GFOLDConfig& cfg) {
    std::ostringstream oss;
    oss << std::setprecision(10) << std::fixed;
    oss << "tf=" << cfg.tf
        << ";g0=" << cfg.g0
        << ";Isp=" << cfg.Isp
        << ";T_max=" << cfg.T_max
        << ";throt=[" << cfg.throttle_min << "|" << cfg.throttle_max << "]"
        << ";m0=" << cfg.m0
        << ";r0=[" << cfg.r0[0] << "|" << cfg.r0[1] << "|" << cfg.r0[2] << "]"
        << ";v0=[" << cfg.v0[0] << "|" << cfg.v0[1] << "|" << cfg.v0[2] << "]"
        << ";glide=" << cfg.glide_slope_deg
        << ";theta=" << cfg.max_angle_deg
        << ";steps=" << cfg.steps
        << ";solver_n=" << cfg.solver_n
        << ";elapsed_time=" << cfg.elapsed_time;
    return oss.str();
}

static bool write_run_csv(const fs::path& out_path, const std::vector<CsvLogRow>& rows) {
    auto parse_mode1_sheet_index = [](const std::string& name, int& idx_out) -> bool {
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
    };

    std::vector<std::string> unique_sheets;
    unique_sheets.reserve(rows.size());
    for (const auto& r : rows) {
        const std::string& name = r.sheet;
        if (name.empty()) continue;
        bool found = false;
        for (const auto& s : unique_sheets) {
            if (s == name) {
                found = true;
                break;
            }
        }
        if (!found) unique_sheets.push_back(name);
    }

    std::vector<std::string> ordered_sheets;
    ordered_sheets.reserve(unique_sheets.size());
    auto has_sheet = [&](const std::string& name) {
        for (const auto& s : unique_sheets) {
            if (s == name) return true;
        }
        return false;
    };
    auto push_sheet_once = [&](const std::string& name) {
        for (const auto& s : ordered_sheets) {
            if (s == name) return;
        }
        ordered_sheets.push_back(name);
    };

    if (has_sheet("mode0")) push_sheet_once("mode0");

    std::vector<std::pair<int, std::string>> mode1_sheets;
    for (const auto& s : unique_sheets) {
        int idx = 0;
        if (parse_mode1_sheet_index(s, idx)) {
            mode1_sheets.push_back({idx, s});
        }
    }
    std::sort(mode1_sheets.begin(), mode1_sheets.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    for (const auto& p : mode1_sheets) push_sheet_once(p.second);

    for (const auto& s : unique_sheets) {
        if (s == "mode0" || s == "receive") continue;
        int idx = 0;
        if (parse_mode1_sheet_index(s, idx)) continue;
        push_sheet_once(s);
    }
    if (has_sheet("receive")) push_sheet_once("receive");

    std::ostringstream oss;
    oss << std::setprecision(10) << std::fixed;
    auto emit_sheet = [&](const std::string& sheet_name) {
        oss << "sheet=" << sheet_name << "\n";
        oss << "solve_seq,index,solver_n,elapsed_time,u_x,u_y,u_z,r_x,r_y,r_z,v_x,v_y,v_z,cfg,note\n";
        bool cfg_written = false;
        for (const auto& r : rows) {
            if (r.sheet != sheet_name) continue;
            oss << r.solve_seq << ",";
            oss << r.index << ",";
            if (r.solver_n > 0) oss << r.solver_n;
            oss << ",";
            write_csv_value(oss, r.elapsed_time); oss << ",";
            write_csv_value(oss, r.ux); oss << ",";
            write_csv_value(oss, r.uy); oss << ",";
            write_csv_value(oss, r.uz); oss << ",";
            write_csv_value(oss, r.rx); oss << ",";
            write_csv_value(oss, r.ry); oss << ",";
            write_csv_value(oss, r.rz); oss << ",";
            write_csv_value(oss, r.vx); oss << ",";
            write_csv_value(oss, r.vy); oss << ",";
            write_csv_value(oss, r.vz); oss << ",";
            if (!cfg_written && !r.cfg.empty()) {
                oss << r.cfg;
                cfg_written = true;
            }
            oss << ",";
            oss << r.note << "\n";
        }
        oss << "\n";
    };
    for (const auto& s : ordered_sheets) emit_sheet(s);
    return atomic_write(out_path, oss.str());
}

static std::vector<CsvLogRow> build_solver_rows(
    const std::string& sheet,
    long long solve_seq,
    int solver_n,
    const std::vector<double>& t,
    const std::vector<double>& ux,
    const std::vector<double>& uy,
    const std::vector<double>& uz,
    const std::vector<double>& rx,
    const std::vector<double>& ry,
    const std::vector<double>& rz,
    const std::vector<double>& vx,
    const std::vector<double>& vy,
    const std::vector<double>& vz,
    const std::string& cfg_text,
    const std::string& note) {
    std::vector<CsvLogRow> out;
    const int steps = static_cast<int>(t.size());
    if (steps <= 0) return out;
    if (static_cast<int>(ux.size()) != steps ||
        static_cast<int>(uy.size()) != steps ||
        static_cast<int>(uz.size()) != steps ||
        static_cast<int>(rx.size()) != steps ||
        static_cast<int>(ry.size()) != steps ||
        static_cast<int>(rz.size()) != steps ||
        static_cast<int>(vx.size()) != steps ||
        static_cast<int>(vy.size()) != steps ||
        static_cast<int>(vz.size()) != steps) {
        return out;
    }
    out.reserve(static_cast<size_t>(steps));
    for (int i = 0; i < steps; ++i) {
        CsvLogRow row;
        row.sheet = sheet;
        row.solve_seq = solve_seq;
        row.index = i;
        row.solver_n = solver_n;
        row.elapsed_time = t[static_cast<size_t>(i)];
        row.ux = ux[i];
        row.uy = uy[i];
        row.uz = uz[i];
        row.rx = rx[i];
        row.ry = ry[i];
        row.rz = rz[i];
        row.vx = vx[i];
        row.vy = vy[i];
        row.vz = vz[i];
        row.cfg = cfg_text;
        row.note = note;
        out.push_back(std::move(row));
    }
    return out;
}

static std::string make_timestamp_csv_name() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tmv{};
#ifdef _WIN32
    localtime_s(&tmv, &t);
#else
    localtime_r(&t, &tmv);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tmv);
    std::ostringstream oss;
    oss << buf << "_" << std::setw(3) << std::setfill('0') << ms.count() << ".csv";
    return oss.str();
}

static bool populate_cfg_from_tokens(const std::vector<std::string>& tok, GFOLDConfig& cfg, double* base_time_out) {
    // Expect at least: MODE,E,N,U,VE,VN,VU,SPEED,THRUST_MAX,ISP,MASS_WET
    if (tok.size() < 11) return false;
    // Full optional fields (mode0):
    // [11]=THROT1, [12]=THROT2, [13]=THETA_DEG, [14]=YGS_DEG, [15]=ELAPSED_SEC
    // Compact fields (mode1/info/end):
    // [11]=ELAPSED_SEC
    try {
        // Solver assumes x-axis is altitude. Map ENU accordingly:
        cfg.r0[0] = std::stod(tok[1]); // U -> x (altitude)
        cfg.r0[1] = std::stod(tok[2]); // N -> y
        cfg.r0[2] = std::stod(tok[3]); // E -> z

        cfg.v0[0] = std::stod(tok[4]); // VU -> vx
        cfg.v0[1] = std::stod(tok[5]); // VN -> vy
        cfg.v0[2] = std::stod(tok[6]); // VE -> vz

        cfg.T_max = std::stod(tok[8]) * 1000.0;   // kN -> N
        cfg.Isp   = std::stod(tok[9]);            // s
        cfg.m0    = std::stod(tok[10]) * 1000.0;  // t -> kg

        const bool has_guidance_fields = (tok.size() > 14);
        if (has_guidance_fields) {
            cfg.throttle_min = std::stod(tok[11]);
            cfg.throttle_max = std::stod(tok[12]);
            cfg.max_angle_deg = std::stod(tok[13]);
            cfg.glide_slope_deg = std::stod(tok[14]);
        }
        if (base_time_out) *base_time_out = 0.0;
        cfg.elapsed_time = 0.0;
        // Optional: elapsed time from kOS (full index 15, compact index 11)
        if (tok.size() > 15) {
            const double et = std::stod(tok[15]);
            if (base_time_out) *base_time_out = et;
            cfg.elapsed_time = et;
        } else if (!has_guidance_fields && tok.size() > 11) {
            const double et = std::stod(tok[11]);
            if (base_time_out) *base_time_out = et;
            cfg.elapsed_time = et;
        }
    } catch (...) {
        return false;
    }
    return true;
}

struct TrajectoryCache {
    int steps = 0;
    double tf = 0.0;
    std::vector<double> t;
    std::vector<double> ux;
    std::vector<double> uy;
    std::vector<double> uz;
    std::vector<double> vx;
    std::vector<double> vy;
    std::vector<double> vz;
    std::vector<double> rx;
    std::vector<double> ry;
    std::vector<double> rz;
    bool valid = false;
};

static int closest_prior_index_by_r(const TrajectoryCache& traj, const double r0[3]) {
    if (!traj.valid || traj.steps <= 0) return 0;
    if (static_cast<int>(traj.rx.size()) < traj.steps ||
        static_cast<int>(traj.ry.size()) < traj.steps ||
        static_cast<int>(traj.rz.size()) < traj.steps) {
        return 0;
    }
    int best_idx = 0;
    double best_d2 = (std::numeric_limits<double>::max)();
    for (int i = 0; i < traj.steps; ++i) {
        const double dx = traj.rx[i] - r0[0];
        const double dy = traj.ry[i] - r0[1];
        const double dz = traj.rz[i] - r0[2];
        const double d2 = dx * dx + dy * dy + dz * dz;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_idx = i;
        }
    }
    return best_idx;
}

static int closest_prior_index_by_v(const TrajectoryCache& traj, const double v0[3]) {
    if (!traj.valid || traj.steps <= 0) return 0;
    if (static_cast<int>(traj.vx.size()) < traj.steps ||
        static_cast<int>(traj.vy.size()) < traj.steps ||
        static_cast<int>(traj.vz.size()) < traj.steps) {
        return 0;
    }
    int best_idx = 0;
    double best_d2 = (std::numeric_limits<double>::max)();
    for (int i = 0; i < traj.steps; ++i) {
        const double dx = traj.vx[i] - v0[0];
        const double dy = traj.vy[i] - v0[1];
        const double dz = traj.vz[i] - v0[2];
        const double d2 = dx * dx + dy * dy + dz * dz;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_idx = i;
        }
    }
    return best_idx;
}

static int choose_mode1_solver_n(double remaining_tf, double recompute_time) {
    static const int kCandidates[] = {100, 50, 25, 10};
    int selected = kCandidates[0];
    for (int n : kCandidates) {
        selected = n;
        const double tf_window = remaining_tf * 10.0 / static_cast<double>(n);
        if (tf_window >= recompute_time || n == 10) break;
    }
    return selected;
}

static void apply_mode1_n_policy(GFOLDConfig& cfg, double recompute_time) {
    const int solver_n = choose_mode1_solver_n(cfg.tf, recompute_time);
    cfg.steps = solver_n;
    cfg.solver_n = solver_n;
}

static std::vector<int> build_sample_indices(int steps, int max_count) {
    std::vector<int> indices;
    if (steps <= 0 || max_count <= 0) return indices;
    const int count = std::min(steps, max_count);
    indices.reserve(static_cast<size_t>(count));
    for (int idx = 0; idx < count; ++idx) indices.push_back(idx);
    return indices;
}

static void apply_tf_rules(
    GFOLDConfig& cfg,
    double nominal_throttle_max,
    double nominal_glide_slope_deg) {
    cfg.throttle_max = nominal_throttle_max;
    cfg.glide_slope_deg = nominal_glide_slope_deg;
}

static void log_mode1_cfg(const GFOLDConfig& cfg) {
    (void)cfg;
}

int main() {
    // Adjust this path to your KSP Scripts folder if needed.
    const fs::path send_path =
        R"(C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\Ships\Script\send.txt)";
    const fs::path recv_path =
        R"(C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\Ships\Script\receive.txt)";

    // Clear old content to avoid stale reads
    {
        std::error_code ec;
        fs::remove(send_path, ec);
        std::ofstream clear(send_path, std::ios::trunc);
    }

    bool has_best_tf = false;
    double best_tf = 0.0;
    bool has_mode0_best_m = false;
    double mode0_best_m = 0.0;
    bool has_mode0_elapsed_sec = false;
    double mode0_elapsed_sec = 0.0;
    std::vector<double> mode0_last_rx;
    std::vector<double> mode0_last_ry;
    std::vector<double> mode0_last_rz;
    std::vector<double> mode0_last_vx;
    std::vector<double> mode0_last_vy;
    std::vector<double> mode0_last_vz;
    std::vector<double> mode0_last_m_traj;
    bool has_mode0_guidance = false;
    double mode0_throttle_min = 0.2;
    double mode0_throttle_max = 0.8;
    double mode0_theta_deg = 45.0;
    double mode0_ygs_deg = 30.0;
    bool fallback_enabled = true;
    bool mode1_solver_enabled = true;
    TrajectoryCache last_traj;
    double last_traj_solve_elapsed_sec = std::numeric_limits<double>::quiet_NaN();
    bool has_fail_solver_cfg = false;
    GFOLDConfig fail_solver_cfg;
    double fail_solver_elapsed_sec = std::numeric_limits<double>::quiet_NaN();
    std::string fail_solver_state;
    std::string fail_fallback_state;
    bool lock_last_traj_after_final_fail = false;
    std::vector<std::array<double, 3>> recv_r_points;
    std::vector<std::array<double, 4>> recv_v_enu_t_points;
    std::vector<std::array<double, 7>> recv_info_rv_t_points;
    bool post_fail_capture_active = false;
    double post_fail_start_elapsed_sec = std::numeric_limits<double>::quiet_NaN();
    double post_fail_recv_cut_elapsed_sec = std::numeric_limits<double>::quiet_NaN();
    int post_fail_last_start_idx = -1;
    std::vector<std::array<double, 3>> post_fail_recv_r_points;
    std::vector<std::array<double, 4>> post_fail_recv_v_xyz_t_points;
    bool end_plot_launched = false;
    constexpr int max_lines = 10;
    constexpr double recompute_time = 1.0+0.3;
    std::vector<InfoEndRvm> info_end_rvm_list;
    long long mode0_solve_seq = 0;
    long long mode1_solve_seq = 0;
    long long recv_row_seq = 0;

    std::mutex queue_mu;
    std::condition_variable queue_cv;
    std::deque<std::string> line_queue;

    std::mutex csv_log_mu;
    std::condition_variable csv_log_cv;
    std::deque<CsvLogTask> csv_log_queue;
    std::thread csv_log_thread([&]() {
        std::vector<CsvLogRow> rows;
        while (true) {
            CsvLogTask task;
            {
                std::unique_lock<std::mutex> lk(csv_log_mu);
                csv_log_cv.wait(lk, [&]() { return !csv_log_queue.empty(); });
                task = std::move(csv_log_queue.front());
                csv_log_queue.pop_front();
            }

            bool ok = true;
            if (task.type == CsvLogTaskType::AppendRows) {
                for (auto& row : task.rows) rows.push_back(std::move(row));
            } else if (task.type == CsvLogTaskType::ResetRun) {
                rows.clear();
            } else if (task.type == CsvLogTaskType::FlushToFile) {
                ok = write_run_csv(task.filename, rows);
            } else if (task.type == CsvLogTaskType::Stop) {
                if (task.done) task.done->set_value(true);
                break;
            }

            if (task.done) task.done->set_value(ok);
        }
    });
    csv_log_thread.detach();

    auto enqueue_log_rows = [&](std::vector<CsvLogRow>&& rows) {
        if (rows.empty()) return;
        CsvLogTask task;
        task.type = CsvLogTaskType::AppendRows;
        task.rows = std::move(rows);
        {
            std::lock_guard<std::mutex> lk(csv_log_mu);
            csv_log_queue.push_back(std::move(task));
        }
        csv_log_cv.notify_one();
    };

    auto reset_log_rows = [&]() {
        CsvLogTask task;
        task.type = CsvLogTaskType::ResetRun;
        {
            std::lock_guard<std::mutex> lk(csv_log_mu);
            csv_log_queue.push_back(std::move(task));
        }
        csv_log_cv.notify_one();
    };

    auto flush_log_rows = [&](const std::string& filename) -> bool {
        CsvLogTask task;
        task.type = CsvLogTaskType::FlushToFile;
        task.filename = filename;
        task.done = std::make_shared<std::promise<bool>>();
        std::future<bool> fut = task.done->get_future();
        {
            std::lock_guard<std::mutex> lk(csv_log_mu);
            csv_log_queue.push_back(std::move(task));
        }
        csv_log_cv.notify_one();
        return fut.get();
    };

    std::thread recv_thread([&]() {
        while (true) {
            std::string line = consume_first_line(send_path);
            if (!line.empty()) {
                {
                    std::lock_guard<std::mutex> lk(queue_mu);
                    line_queue.push_back(std::move(line));
                }
                queue_cv.notify_one();
            } else {
                std::this_thread::sleep_for(1ms);
            }
        }
    });
    recv_thread.detach();

    while (true) {
        std::string line;
        {
            std::unique_lock<std::mutex> lk(queue_mu);
            queue_cv.wait(lk, [&]() { return !line_queue.empty(); });
            line = std::move(line_queue.front());
            line_queue.pop_front();
        }

        auto tok = split_csv(line);
        if (tok.empty()) continue;

        const IncomingType incoming = parse_incoming_type(tok[0]);
        if (incoming == IncomingType::Unknown) continue;
        if (incoming == IncomingType::Mode0) {
            // Start of a new run: clear asynchronous CSV buffers before collecting new rows.
            reset_log_rows();
            mode0_solve_seq = 0;
            mode1_solve_seq = 0;
            recv_row_seq = 0;
        }
        std::array<double, 3> r_xyz{};
        const bool has_r_xyz = try_parse_r_xyz(tok, r_xyz);
        if (has_r_xyz) {
            recv_r_points.push_back(r_xyz);
        }
        std::array<double, 4> v_enu_t{};
        const bool has_v_enu_t = try_parse_v_enu_t(tok, v_enu_t);
        if (has_v_enu_t) {
            recv_v_enu_t_points.push_back(v_enu_t);
        }
        if (has_r_xyz && has_v_enu_t) {
            CsvLogRow row;
            row.sheet = "receive";
            row.solve_seq = recv_row_seq++;
            row.index = 0;
            row.elapsed_time = v_enu_t[0];
            row.rx = r_xyz[0];
            row.ry = r_xyz[1];
            row.rz = r_xyz[2];
            row.vx = v_enu_t[1];
            row.vy = v_enu_t[2];
            row.vz = v_enu_t[3];
            row.note = "incoming_" + tok[0];
            std::vector<CsvLogRow> batch;
            batch.push_back(std::move(row));
            enqueue_log_rows(std::move(batch));
        }
        if (incoming == IncomingType::Info || incoming == IncomingType::End) {
            if (has_r_xyz && has_v_enu_t) {
                recv_info_rv_t_points.push_back({
                    v_enu_t[0], r_xyz[0], r_xyz[1], r_xyz[2], v_enu_t[1], v_enu_t[2], v_enu_t[3]
                });
            }
            InfoEndRvm sample;
            (void)parse_info_end_rvm(tok, sample);
            info_end_rvm_list.push_back(sample);
            if (incoming == IncomingType::End && !end_plot_launched) {
                double end_time_sec = std::numeric_limits<double>::quiet_NaN();
                const bool has_end_time = try_parse_double_at(tok, 15, end_time_sec);
                const double n_err = sample.r1;
                const double e_err = sample.r2;
                const double h_err = std::sqrt(n_err * n_err + e_err * e_err);
                const double end_m_kg = sample.m * 1000.0; // kOS sends MASS_WET in tons
                const bool has_mode0_r0 =
                    !mode0_last_rx.empty() &&
                    !mode0_last_ry.empty() &&
                    !mode0_last_rz.empty();
                const bool has_recv_r0 = !recv_r_points.empty();
                const bool has_mode0_v0 =
                    !mode0_last_vx.empty() &&
                    !mode0_last_vy.empty() &&
                    !mode0_last_vz.empty();
                const bool has_recv_v0 = !recv_v_enu_t_points.empty();
                double dr_up = std::numeric_limits<double>::quiet_NaN();
                double dr_north = std::numeric_limits<double>::quiet_NaN();
                double dr_east = std::numeric_limits<double>::quiet_NaN();
                double dv2_up = std::numeric_limits<double>::quiet_NaN();
                double dv2_north = std::numeric_limits<double>::quiet_NaN();
                double dv2_east = std::numeric_limits<double>::quiet_NaN();
                if (has_mode0_r0 && has_recv_r0) {
                    dr_up = recv_r_points.front()[0] - mode0_last_rx.front();
                    dr_north = recv_r_points.front()[1] - mode0_last_ry.front();
                    dr_east = recv_r_points.front()[2] - mode0_last_rz.front();
                }
                if (has_mode0_v0 && has_recv_v0) {
                    const double recv_up = recv_v_enu_t_points.front()[1];
                    const double recv_north = recv_v_enu_t_points.front()[2];
                    const double recv_east = recv_v_enu_t_points.front()[3];
                    const double mode0_up = mode0_last_vx.front();
                    const double mode0_north = mode0_last_vy.front();
                    const double mode0_east = mode0_last_vz.front();
                    dv2_up = (recv_up * recv_up) - (mode0_up * mode0_up);
                    dv2_north = (recv_north * recv_north) - (mode0_north * mode0_north);
                    dv2_east = (recv_east * recv_east) - (mode0_east * mode0_east);
                }

                std::cout << std::fixed << std::setprecision(6);
                std::cout << "[end] best_m="
                          << (has_mode0_best_m ? mode0_best_m : -1.0)
                          << " mode0_time="
                          << (has_mode0_elapsed_sec ? mode0_elapsed_sec : -1.0)
                          << " end_m=" << end_m_kg
                          << " horiz_err_N=" << n_err
                          << " horiz_err_E=" << e_err
                          << " horiz_err_mag=" << h_err;
                if (has_mode0_r0 && has_recv_r0) {
                    std::cout << " first_r_err=["
                              << dr_up << "," << dr_north << "," << dr_east << "]";
                } else {
                    std::cout << " first_r_err=n/a";
                }
                if (has_mode0_v0 && has_recv_v0) {
                    std::cout << " first_v2_err=["
                              << dv2_up << "," << dv2_north << "," << dv2_east << "]";
                } else {
                    std::cout << " first_v2_err=n/a";
                }
                if (has_end_time) {
                    std::cout << " landing_t=" << end_time_sec;
                } else {
                    std::cout << " landing_t=n/a";
                }
                std::cout << "\n";

                const std::string out_csv_name = make_timestamp_csv_name();
                const bool flushed = flush_log_rows(out_csv_name);
                if (flushed) {
                    std::cout << "[end] csv_saved=" << out_csv_name << "\n";
                    if (!plot_from_run_csv(out_csv_name)) {
                        std::cout << "[end] plot_skipped=" << out_csv_name << "\n";
                    }
                } else {
                    std::cout << "[end] csv_save_failed=" << out_csv_name << "\n";
                }
                end_plot_launched = true;
            }
            continue;
        }

        const int mode = (incoming == IncomingType::Mode0) ? 0 : 1;

        if (mode == 0) {
            // A new mode0 request starts a fresh run; drop previous run history.
            recv_r_points.clear();
            recv_v_enu_t_points.clear();
            recv_info_rv_t_points.clear();
            info_end_rvm_list.clear();
            end_plot_launched = false;
            has_mode0_best_m = false;
            mode0_best_m = 0.0;
            has_mode0_elapsed_sec = false;
            mode0_elapsed_sec = 0.0;
            mode0_last_rx.clear();
            mode0_last_ry.clear();
            mode0_last_rz.clear();
            mode0_last_vx.clear();
            mode0_last_vy.clear();
            mode0_last_vz.clear();
            mode0_last_m_traj.clear();
            has_mode0_guidance = false;
            has_fail_solver_cfg = false;
            fail_solver_cfg = GFOLDConfig{};
            fail_solver_elapsed_sec = std::numeric_limits<double>::quiet_NaN();
            fail_solver_state.clear();
            fail_fallback_state.clear();
            lock_last_traj_after_final_fail = false;
            post_fail_capture_active = false;
            post_fail_start_elapsed_sec = std::numeric_limits<double>::quiet_NaN();
            post_fail_recv_cut_elapsed_sec = std::numeric_limits<double>::quiet_NaN();
            post_fail_last_start_idx = -1;
            post_fail_recv_r_points.clear();
            post_fail_recv_v_xyz_t_points.clear();

            GFOLDConfig cfg; // start with defaults
            cfg.tf = 60.0;   // placeholder; actual tf is searched over
            cfg.g0 = 9.81;
            cfg.elapsed_time = 0.0;
            if (!populate_cfg_from_tokens(tok, cfg, nullptr)) {
//                std::cerr << "Parse error on line: " << line << "\n";
                write_request_mode0(recv_path, "parse_error");
                continue;
            }
            cfg.elapsed_time = 0.0;
            mode0_throttle_min = cfg.throttle_min;
            mode0_throttle_max = cfg.throttle_max;
            mode0_theta_deg = cfg.max_angle_deg;
            mode0_ygs_deg = cfg.glide_slope_deg;
            has_mode0_guidance = true;

            const double L = 10.0, R = 90.0;
            SearchResult res = find_best_tf(cfg, L, R, 10, true);

            if (!res.feasible) {
                write_request_mode0(recv_path, "infeasible");
                continue;
            }

            has_best_tf = true;
            best_tf = res.best_tf;
            mode0_best_m = res.best_m;
            has_mode0_best_m = true;
            has_mode0_elapsed_sec = true;
            mode0_elapsed_sec = res.elapsed_sec;
            if (res.has_last_traj) {
                mode0_last_rx = res.last_rx;
                mode0_last_ry = res.last_ry;
                mode0_last_rz = res.last_rz;
                mode0_last_vx = res.last_vx;
                mode0_last_vy = res.last_vy;
                mode0_last_vz = res.last_vz;
                mode0_last_m_traj = res.last_m_traj;

                GFOLDConfig mode0_cfg_for_log = cfg;
                mode0_cfg_for_log.tf = (res.last_tf > 0.0) ? res.last_tf : res.best_tf;
                const std::string mode0_cfg_text = format_solver_cfg(mode0_cfg_for_log);
                auto rows = build_solver_rows(
                    "mode0",
                    mode0_solve_seq++,
                    mode0_cfg_for_log.solver_n,
                    res.last_t,
                    res.last_ux,
                    res.last_uy,
                    res.last_uz,
                    res.last_rx,
                    res.last_ry,
                    res.last_rz,
                    res.last_vx,
                    res.last_vy,
                    res.last_vz,
                    mode0_cfg_text,
                    "mode0_last_feasible");
                enqueue_log_rows(std::move(rows));
            }
            fallback_enabled = true;
            mode1_solver_enabled = true;
            last_traj.valid = false;
            last_traj_solve_elapsed_sec = std::numeric_limits<double>::quiet_NaN();
            // Request kOS to send fresh state as mode1 (fixed-format marker)
            {
                write_compute_finish(recv_path, 1);
            }
        } else if (mode == 1 && has_best_tf) {
            // Populate cfg from current state, reuse best_tf
            GFOLDConfig cfg;
            cfg.tf = best_tf;
            cfg.g0 = 9.81;
            double base_time = 0.0;
            if (!populate_cfg_from_tokens(tok, cfg, &base_time)) {
//                std::cerr << "Parse error on mode1 line: " << line << "\n";
                continue;
            }
            if (has_mode0_guidance) {
                cfg.throttle_min = mode0_throttle_min;
                cfg.throttle_max = mode0_throttle_max;
                cfg.max_angle_deg = mode0_theta_deg;
                cfg.glide_slope_deg = mode0_ygs_deg;
            }

            if (last_traj.valid && last_traj.steps > 0 && last_traj.tf > 0.0) {
                const int idx = closest_prior_index_by_r(last_traj, cfg.r0);
                const double dt_prev = last_traj.tf / static_cast<double>(last_traj.steps);
                const double rem_tf = dt_prev * static_cast<double>(last_traj.steps - idx);
                if (rem_tf > 0.0) cfg.tf = rem_tf;
            }
            const bool tf_short = (cfg.tf < 5.0);
            const double nominal_throttle_max = cfg.throttle_max;
            const double nominal_glide_slope_deg = cfg.glide_slope_deg;
            apply_tf_rules(cfg, nominal_throttle_max, nominal_glide_slope_deg);
            apply_mode1_n_policy(cfg, recompute_time);
            log_mode1_cfg(cfg);

            GFOLDSolver solver(cfg);
            bool ok = false;
            std::string solver_state = "disabled";
            std::string fallback_state = "not_used";
            TrajectoryCache active_traj;
            if (mode1_solver_enabled) {
                int mode1_solver_n = cfg.solver_n;
                ok = solver.solve(mode1_solver_n);
                solver_state = ok ? "ok" : ("fail(" + std::to_string(solver.status()) + ")");
                if (!ok) {
                    const bool allow_fallback = fallback_enabled || tf_short;
                    if (allow_fallback) {
                        if (tf_short) {
                            fallback_state = "search_throt2";
                            ThrottleSearchResult retry_throt2 =
                                find_min_feasible_throttle_max(cfg, cfg.throttle_max, 1.0, 8);
                            if (!retry_throt2.feasible) {
                                const bool at_min_solver_n = (cfg.solver_n <= 10);
                                fallback_state = at_min_solver_n ? "infeasible_throt2_min_n" : "infeasible_throt2";
                                fallback_enabled = false;
                            } else {
                                fallback_state = "ok_throt2";
                                cfg.throttle_max = retry_throt2.best_throttle_max;
                                mode0_throttle_max = cfg.throttle_max;
                                has_mode0_guidance = true;
                                apply_tf_rules(cfg, cfg.throttle_max, nominal_glide_slope_deg);
                                apply_mode1_n_policy(cfg, recompute_time);
                                log_mode1_cfg(cfg);
                                solver.set_config(cfg);
                                mode1_solver_n = cfg.solver_n;
                                ok = solver.solve(mode1_solver_n);
                                if (!ok) {
                                    solver_state = "fail(" + std::to_string(solver.status()) + ")";
                                    fallback_state = "solve_fail_throt2";
                                    fallback_enabled = false;
                                } else {
                                    solver_state = "ok";
                                }
                            }
                        } else {
                            const double tf_min = cfg.tf;
                            const double tf_max = cfg.tf + 1.5;
                            fallback_state = "search";

                            GFOLDConfig search_cfg = cfg;
                            search_cfg.throttle_max = cfg.throttle_max;
                            search_cfg.glide_slope_deg = nominal_glide_slope_deg;
                            SearchResult retry = find_best_tf(search_cfg, tf_min, tf_max, 4, false);
                            if (!retry.feasible) {
                                fallback_state = "infeasible";
                                const bool at_min_solver_n = (cfg.solver_n <= 10);
                                if (at_min_solver_n) {
                                    fallback_enabled = false;
//                                std::cerr << "[mode1] fallback infeasible at min solver n="
//                                          << cfg.solver_n
//                                          << ", return COMPUTE_FINISH,2\n";
                                    write_compute_finish(recv_path, 2);
                                    continue;
                                }
                                fallback_enabled = false;
//                            std::cerr << "[mode1] fallback find_best_tf infeasible in ["
//                                      << tf_min << ", " << tf_max
//                                      << "], disable fallback and solver, reuse last trajectory\n";
                            } else {
                                fallback_state = "ok";
                                cfg.tf = retry.best_tf;
                                const double retry_throttle_max = cfg.throttle_max;
                                apply_tf_rules(cfg, retry_throttle_max, nominal_glide_slope_deg);
                                apply_mode1_n_policy(cfg, recompute_time);
                                log_mode1_cfg(cfg);
                                best_tf = retry.best_tf;
                                solver.set_config(cfg);
                                mode1_solver_n = cfg.solver_n;
                                ok = solver.solve(mode1_solver_n);
                                if (!ok) {
                                    solver_state = "fail(" + std::to_string(solver.status()) + ")";
                                    fallback_state = "solve_fail";
                                    fallback_enabled = false;
//                                std::cerr << "[mode1] fallback solve failed, status="
//                                          << solver.status() << " tf=" << cfg.tf
//                                          << ", disable fallback and solver, reuse last trajectory\n";
                                } else {
                                    solver_state = "ok";
                                }
                            }
                        }
                    } else {
                        fallback_state = "disabled";
                    }
                }
            } else {
                solver_state = "disabled";
                fallback_state = "disabled";
            }

            if (!ok &&
                !post_fail_capture_active &&
                !lock_last_traj_after_final_fail &&
                last_traj.valid &&
                last_traj.steps > 0 &&
                last_traj.tf > 0.0 &&
                static_cast<int>(last_traj.t.size()) == last_traj.steps &&
                static_cast<int>(last_traj.vx.size()) == last_traj.steps &&
                static_cast<int>(last_traj.vy.size()) == last_traj.steps &&
                static_cast<int>(last_traj.vz.size()) == last_traj.steps &&
                static_cast<int>(last_traj.rx.size()) == last_traj.steps &&
                static_cast<int>(last_traj.ry.size()) == last_traj.steps &&
                static_cast<int>(last_traj.rz.size()) == last_traj.steps) {
                const int start_idx = closest_prior_index_by_r(last_traj, cfg.r0);
                if (start_idx >= 0 && start_idx < last_traj.steps) {
                    const int rem_steps = last_traj.steps - start_idx;
                    if (rem_steps > 0) {
                        const double dt_prev = last_traj.tf / static_cast<double>(last_traj.steps);
                        post_fail_last_start_idx = start_idx;

                        post_fail_recv_r_points.clear();
                        post_fail_recv_v_xyz_t_points.clear();
                        post_fail_start_elapsed_sec =
                            std::isfinite(last_traj_solve_elapsed_sec) ? last_traj_solve_elapsed_sec : base_time;
                        post_fail_recv_cut_elapsed_sec =
                            post_fail_start_elapsed_sec + (static_cast<double>(start_idx) * dt_prev);
                        has_fail_solver_cfg = true;
                        fail_solver_cfg = cfg;
                        fail_solver_elapsed_sec = base_time;
                        fail_solver_state = solver_state;
                        fail_fallback_state = fallback_state;
                        post_fail_capture_active = true;
                        fallback_enabled = false;
                    }
                }
            }

            if (ok) {
                const int steps = cfg.steps;
                GFOLDSolution sol = solver.solution();
                if (sol.steps != steps ||
                    static_cast<int>(sol.t.size()) != steps ||
                    static_cast<int>(sol.ux.size()) != steps ||
                    static_cast<int>(sol.uy.size()) != steps ||
                    static_cast<int>(sol.uz.size()) != steps ||
                    static_cast<int>(sol.vx.size()) != steps ||
                    static_cast<int>(sol.vy.size()) != steps ||
                    static_cast<int>(sol.vz.size()) != steps ||
                    static_cast<int>(sol.rx.size()) != steps ||
                    static_cast<int>(sol.ry.size()) != steps ||
                    static_cast<int>(sol.rz.size()) != steps) {
                    write_compute_finish(recv_path, 2);
                    continue;
                }

                active_traj.steps = steps;
                active_traj.tf = cfg.tf;
                active_traj.t = std::move(sol.t);
                active_traj.ux = std::move(sol.ux);
                active_traj.uy = std::move(sol.uy);
                active_traj.uz = std::move(sol.uz);
                active_traj.vx = std::move(sol.vx);
                active_traj.vy = std::move(sol.vy);
                active_traj.vz = std::move(sol.vz);
                active_traj.rx = std::move(sol.rx);
                active_traj.ry = std::move(sol.ry);
                active_traj.rz = std::move(sol.rz);
                active_traj.valid = true;
                // Re-enable fallback once a fresh solver result is valid.
                fallback_enabled = true;
            } else {
                const bool cache_ready =
                    last_traj.valid &&
                    last_traj.steps > 0 &&
                    last_traj.tf > 0.0 &&
                    static_cast<int>(last_traj.t.size()) == last_traj.steps &&
                    static_cast<int>(last_traj.ux.size()) == last_traj.steps &&
                    static_cast<int>(last_traj.uy.size()) == last_traj.steps &&
                    static_cast<int>(last_traj.uz.size()) == last_traj.steps &&
                    static_cast<int>(last_traj.vx.size()) == last_traj.steps &&
                    static_cast<int>(last_traj.vy.size()) == last_traj.steps &&
                    static_cast<int>(last_traj.vz.size()) == last_traj.steps &&
                    static_cast<int>(last_traj.rx.size()) == last_traj.steps &&
                    static_cast<int>(last_traj.ry.size()) == last_traj.steps &&
                    static_cast<int>(last_traj.rz.size()) == last_traj.steps;

                if (!cache_ready) {
//                    std::cerr << "[mode1] fallback failed and no valid cached trajectory\n";
                    write_compute_finish(recv_path, 2);
                    continue;
                }

                const int start_idx = closest_prior_index_by_r(last_traj, cfg.r0);
                if (start_idx < 0 || start_idx >= last_traj.steps) {
//                    std::cerr << "[mode1] invalid cached start_idx=" << start_idx << "\n";
                    write_compute_finish(recv_path, 2);
                    continue;
                }

                const int rem_steps = last_traj.steps - start_idx;
                if (rem_steps <= 0) {
//                    std::cerr << "[mode1] cached trajectory has no remaining steps\n";
                    write_compute_finish(recv_path, 2);
                    continue;
                }

                const double dt_prev = last_traj.tf / static_cast<double>(last_traj.steps);
                active_traj.steps = rem_steps;
                active_traj.tf = dt_prev * static_cast<double>(rem_steps);
                active_traj.t.assign(last_traj.t.begin() + start_idx, last_traj.t.end());
                active_traj.ux.assign(last_traj.ux.begin() + start_idx, last_traj.ux.end());
                active_traj.uy.assign(last_traj.uy.begin() + start_idx, last_traj.uy.end());
                active_traj.uz.assign(last_traj.uz.begin() + start_idx, last_traj.uz.end());
                active_traj.vx.assign(last_traj.vx.begin() + start_idx, last_traj.vx.end());
                active_traj.vy.assign(last_traj.vy.begin() + start_idx, last_traj.vy.end());
                active_traj.vz.assign(last_traj.vz.begin() + start_idx, last_traj.vz.end());
                active_traj.rx.assign(last_traj.rx.begin() + start_idx, last_traj.rx.end());
                active_traj.ry.assign(last_traj.ry.begin() + start_idx, last_traj.ry.end());
                active_traj.rz.assign(last_traj.rz.begin() + start_idx, last_traj.rz.end());
                active_traj.valid = true;

                cfg.tf = active_traj.tf;
            }

            if (!active_traj.valid || active_traj.steps <= 0) {
//                std::cerr << "[mode1] no active trajectory to emit\n";
                write_compute_finish(recv_path, 2);
                continue;
            }

            if (!lock_last_traj_after_final_fail) {
                last_traj = active_traj;
                last_traj_solve_elapsed_sec = active_traj.t.empty() ? base_time : active_traj.t.front();
            }

            {
                const long long this_mode1_seq = mode1_solve_seq++;
                const std::string mode1_sheet_name = "mode1_" + std::to_string(this_mode1_seq + 1);
                const std::string mode1_cfg_text = format_solver_cfg(cfg);
                auto rows = build_solver_rows(
                    mode1_sheet_name,
                    this_mode1_seq,
                    cfg.solver_n,
                    active_traj.t,
                    active_traj.ux,
                    active_traj.uy,
                    active_traj.uz,
                    active_traj.rx,
                    active_traj.ry,
                    active_traj.rz,
                    active_traj.vx,
                    active_traj.vy,
                    active_traj.vz,
                    mode1_cfg_text,
                    ok ? "mode1_solver_ok" : "mode1_reuse_cache");
                enqueue_log_rows(std::move(rows));
            }

            const int steps = active_traj.steps;
            const std::vector<double>& ux = active_traj.ux;
            const std::vector<double>& uy = active_traj.uy;
            const std::vector<double>& uz = active_traj.uz;

            std::ostringstream oss;
            oss << std::setprecision(10) << std::fixed;
            int recv_lines = 0;
            oss << "COMPUTE_FINISH,1\n";
            recv_lines += 1;
            auto emit_indices = build_sample_indices(steps, max_lines);

            std::cout << "\x1B[2J\x1B[H";
            std::cout << std::flush;

            std::cout << std::fixed << std::setprecision(6);
            const double best_m_to_print = has_mode0_best_m ? mode0_best_m : 0.0;
            const int mode1_solver_n = (cfg.solver_n > 0) ? cfg.solver_n : steps;
            const double tf_window = active_traj.tf * 10.0 / static_cast<double>(mode1_solver_n);
            std::cout << "[mode1] solver=" << solver_state
                      << " fallback_state=" << fallback_state
                      << " fallback_enabled=" << (fallback_enabled ? 1 : 0)
                      << " remaining_tf=" << active_traj.tf
                      << " n=" << mode1_solver_n
                      << " tf10_over_n=" << tf_window
                      << " best_m=" << best_m_to_print << "\n";

            for (int idx : emit_indices) {
                const double up = ux[idx];
                const double north = uy[idx];
                const double east = uz[idx];
                const double t_abs = active_traj.t[idx];
                const double t_out = t_abs;
                const double mag = std::sqrt(up * up + north * north + east * east);
                std::cout << "[mode1] U[" << idx << "] up=" << up
                          << " north=" << north
                          << " east=" << east
                          << " mag=" << mag
                          << " t=" << t_out << "\n";
                oss << "U," << up << "," << north << "," << east << "," << t_out << "\n";
                recv_lines += 1;
            }
            oss << "REMAIN_TF," << active_traj.tf << "\n";
            recv_lines += 1;
            if (!atomic_write(recv_path, oss.str())) {
//                std::cerr << "Failed to write receive.txt\n";
            }
        }
    }

    return 0;
}
