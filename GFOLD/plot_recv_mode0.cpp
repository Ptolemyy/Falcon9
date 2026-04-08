#define NOMINMAX
#include <matplotlibcpp17/mplot3d.h>
#include <matplotlibcpp17/pyplot.h>
#include <pybind11/embed.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace py = pybind11;

struct CsvRunRow {
    std::string sheet;
    int solve_seq = 0;
    int index = 0;
    int solver_n = -1;
    double t = std::numeric_limits<double>::quiet_NaN();
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

struct CsvSheetBlock {
    std::string name;
    std::vector<CsvRunRow> rows;
};

struct ParsedRunCsv {
    std::vector<CsvSheetBlock> sheets;
};

struct Series3D {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
};

struct SeriesTV {
    std::vector<double> t;
    std::vector<double> v;
};

struct ErrorNormSeries {
    std::vector<double> dt;
    std::vector<double> pos_norm;
    std::vector<double> vel_norm;
};

struct Mode1SheetRows {
    int idx = 0;
    std::string name;
    bool reuse = false;
    std::vector<CsvRunRow> rows;
};

struct Mode1Segment {
    int idx = 0;
    bool reuse = false;
    std::vector<double> t;
    std::vector<double> rx;
    std::vector<double> ry;
    std::vector<double> rz;
    std::vector<double> vx;
    std::vector<double> vy;
    std::vector<double> vz;
};

static bool starts_with(const std::string& s, const std::string& prefix) {
    return s.rfind(prefix, 0) == 0;
}

static std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) out.push_back(item);
    return out;
}

static int parse_int_or_default(const std::string& s, int def_v = 0) {
    if (s.empty()) return def_v;
    try {
        return std::stoi(s);
    } catch (...) {
        return def_v;
    }
}

static double parse_double_or_nan(const std::string& s) {
    if (s.empty()) return std::numeric_limits<double>::quiet_NaN();
    try {
        return std::stod(s);
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

static bool parse_mode1_sheet_index(const std::string& name, int& idx_out) {
    static const std::string kPrefix = "mode1_";
    if (!starts_with(name, kPrefix)) return false;
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

static ParsedRunCsv load_run_csv(const std::string& path) {
    ParsedRunCsv parsed;
    std::ifstream in(path);
    if (!in) return parsed;

    std::string current_sheet;
    bool skip_header = false;

    auto get_or_add_sheet = [&](const std::string& name) -> CsvSheetBlock& {
        for (auto& s : parsed.sheets) {
            if (s.name == name) return s;
        }
        parsed.sheets.push_back(CsvSheetBlock{name, {}});
        return parsed.sheets.back();
    };

    std::string line;
    while (std::getline(in, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();
        if (line.empty()) continue;

        if (starts_with(line, "sheet=")) {
            current_sheet = line.substr(6);
            skip_header = true;
            continue;
        }
        if (skip_header) {
            skip_header = false;
            continue;
        }
        if (current_sheet.empty()) continue;

        const auto tok = split_csv(line);
        if (tok.size() < 14) continue;

        // New format inserts solver_n after index:
        // old : solve_seq,index,elapsed_time,...,cfg,note
        // new : solve_seq,index,solver_n,elapsed_time,...,cfg,note
        const int off = (tok.size() >= 15) ? 1 : 0;
        if (static_cast<int>(tok.size()) < (14 + off)) continue;

        CsvRunRow row;
        row.sheet = current_sheet;
        row.solve_seq = parse_int_or_default(tok[0], 0);
        row.index = parse_int_or_default(tok[1], 0);
        row.solver_n = (off == 1) ? parse_int_or_default(tok[2], -1) : -1;
        row.t = parse_double_or_nan(tok[2 + off]);
        row.ux = parse_double_or_nan(tok[3 + off]);
        row.uy = parse_double_or_nan(tok[4 + off]);
        row.uz = parse_double_or_nan(tok[5 + off]);
        row.rx = parse_double_or_nan(tok[6 + off]);
        row.ry = parse_double_or_nan(tok[7 + off]);
        row.rz = parse_double_or_nan(tok[8 + off]);
        row.vx = parse_double_or_nan(tok[9 + off]);
        row.vy = parse_double_or_nan(tok[10 + off]);
        row.vz = parse_double_or_nan(tok[11 + off]);
        row.cfg = tok[12 + off];
        row.note = tok[13 + off];

        auto& sheet = get_or_add_sheet(current_sheet);
        sheet.rows.push_back(std::move(row));
    }

    return parsed;
}

static const CsvSheetBlock* find_sheet(const ParsedRunCsv& parsed, const std::string& name) {
    for (const auto& s : parsed.sheets) {
        if (s.name == name) return &s;
    }
    return nullptr;
}

static std::vector<Mode1SheetRows> extract_mode1_sheets(const ParsedRunCsv& parsed) {
    std::vector<Mode1SheetRows> out;
    for (const auto& s : parsed.sheets) {
        int idx = 0;
        if (!parse_mode1_sheet_index(s.name, idx)) continue;
        Mode1SheetRows m;
        m.idx = idx;
        m.name = s.name;
        m.rows = s.rows;
        for (const auto& r : s.rows) {
            if (r.note.find("reuse") != std::string::npos) {
                m.reuse = true;
                break;
            }
        }
        out.push_back(std::move(m));
    }
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.idx < b.idx; });
    return out;
}

static Series3D build_r3d_series(const std::vector<CsvRunRow>& rows) {
    Series3D s;
    for (const auto& r : rows) {
        if (!std::isfinite(r.rx) || !std::isfinite(r.ry) || !std::isfinite(r.rz)) continue;
        s.x.push_back(r.rx);
        s.y.push_back(r.ry);
        s.z.push_back(r.rz);
    }
    return s;
}

static SeriesTV build_tv_series_from_recv(const std::vector<CsvRunRow>& rows, int comp) {
    SeriesTV s;
    for (const auto& r : rows) {
        if (!std::isfinite(r.t)) continue;
        double v = std::numeric_limits<double>::quiet_NaN();
        if (comp == 0) v = r.vx;
        if (comp == 1) v = r.vy;
        if (comp == 2) v = r.vz;
        if (comp == 3) v = r.rx;
        if (comp == 4) v = r.ry;
        if (comp == 5) v = r.rz;
        if (!std::isfinite(v)) continue;
        s.t.push_back(r.t);
        s.v.push_back(v);
    }
    return s;
}

struct RvSample {
    double t = std::numeric_limits<double>::quiet_NaN();
    double rx = std::numeric_limits<double>::quiet_NaN();
    double ry = std::numeric_limits<double>::quiet_NaN();
    double rz = std::numeric_limits<double>::quiet_NaN();
    double vx = std::numeric_limits<double>::quiet_NaN();
    double vy = std::numeric_limits<double>::quiet_NaN();
    double vz = std::numeric_limits<double>::quiet_NaN();
};

static std::vector<RvSample> build_rv_samples(const std::vector<CsvRunRow>& rows) {
    std::vector<RvSample> out;
    out.reserve(rows.size());
    for (const auto& r : rows) {
        if (!std::isfinite(r.t) ||
            !std::isfinite(r.rx) || !std::isfinite(r.ry) || !std::isfinite(r.rz) ||
            !std::isfinite(r.vx) || !std::isfinite(r.vy) || !std::isfinite(r.vz)) {
            continue;
        }
        RvSample s;
        s.t = r.t;
        s.rx = r.rx;
        s.ry = r.ry;
        s.rz = r.rz;
        s.vx = r.vx;
        s.vy = r.vy;
        s.vz = r.vz;
        out.push_back(s);
    }
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.t < b.t; });
    return out;
}

static bool interpolate_rv_at(
    const std::vector<RvSample>& series,
    double t_query,
    std::size_t& cursor,
    RvSample& out) {
    if (series.empty()) return false;
    if (t_query < series.front().t || t_query > series.back().t) return false;

    while (cursor + 1 < series.size() && series[cursor + 1].t < t_query) {
        ++cursor;
    }
    if (cursor >= series.size()) cursor = series.size() - 1;

    const RvSample& s0 = series[cursor];
    if (cursor + 1 >= series.size()) {
        out = s0;
        return true;
    }

    const RvSample& s1 = series[cursor + 1];
    const double dt = s1.t - s0.t;
    if (dt <= 1e-12) {
        out = s0;
        return true;
    }

    double a = (t_query - s0.t) / dt;
    if (a < 0.0) a = 0.0;
    if (a > 1.0) a = 1.0;

    out.t = t_query;
    out.rx = s0.rx + (s1.rx - s0.rx) * a;
    out.ry = s0.ry + (s1.ry - s0.ry) * a;
    out.rz = s0.rz + (s1.rz - s0.rz) * a;
    out.vx = s0.vx + (s1.vx - s0.vx) * a;
    out.vy = s0.vy + (s1.vy - s0.vy) * a;
    out.vz = s0.vz + (s1.vz - s0.vz) * a;
    return true;
}

static ErrorNormSeries build_recv_mode0_error_norm_series(
    const std::vector<CsvRunRow>& recv_rows,
    const std::vector<CsvRunRow>& mode0_rows) {
    ErrorNormSeries out;
    const std::vector<RvSample> recv = build_rv_samples(recv_rows);
    const std::vector<RvSample> mode0 = build_rv_samples(mode0_rows);
    if (recv.empty() || mode0.empty()) return out;

    std::size_t mode0_cursor = 0;
    bool has_t0 = false;
    double t0 = 0.0;
    for (const auto& r : recv) {
        RvSample m;
        if (!interpolate_rv_at(mode0, r.t, mode0_cursor, m)) continue;
        if (!has_t0) {
            t0 = r.t;
            has_t0 = true;
        }
        const double drx = r.rx - m.rx;
        const double dry = r.ry - m.ry;
        const double drz = r.rz - m.rz;
        const double dvx = r.vx - m.vx;
        const double dvy = r.vy - m.vy;
        const double dvz = r.vz - m.vz;
        out.dt.push_back(r.t - t0);
        out.pos_norm.push_back(std::sqrt(drx * drx + dry * dry + drz * drz));
        out.vel_norm.push_back(std::sqrt(dvx * dvx + dvy * dvy + dvz * dvz));
    }
    return out;
}

static std::vector<RvSample> build_rv_samples_from_segment(const Mode1Segment& seg) {
    std::vector<RvSample> out;
    const std::size_t n = std::min(
        seg.t.size(),
        std::min(
            seg.rx.size(),
            std::min(
                seg.ry.size(),
                std::min(
                    seg.rz.size(),
                    std::min(seg.vx.size(), std::min(seg.vy.size(), seg.vz.size()))))));
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (!std::isfinite(seg.t[i]) ||
            !std::isfinite(seg.rx[i]) || !std::isfinite(seg.ry[i]) || !std::isfinite(seg.rz[i]) ||
            !std::isfinite(seg.vx[i]) || !std::isfinite(seg.vy[i]) || !std::isfinite(seg.vz[i])) {
            continue;
        }
        RvSample s;
        s.t = seg.t[i];
        s.rx = seg.rx[i];
        s.ry = seg.ry[i];
        s.rz = seg.rz[i];
        s.vx = seg.vx[i];
        s.vy = seg.vy[i];
        s.vz = seg.vz[i];
        out.push_back(s);
    }
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.t < b.t; });
    return out;
}

static ErrorNormSeries build_recv_segment_error_norm_series(
    const std::vector<CsvRunRow>& recv_rows,
    const std::vector<Mode1Segment>& segments) {
    ErrorNormSeries out;
    const std::vector<RvSample> recv = build_rv_samples(recv_rows);
    if (recv.empty() || segments.empty()) return out;

    std::vector<std::vector<RvSample>> refs;
    refs.reserve(segments.size());
    for (const auto& seg : segments) {
        auto s = build_rv_samples_from_segment(seg);
        if (s.size() < 2) continue;
        refs.push_back(std::move(s));
    }
    if (refs.empty()) return out;

    std::vector<std::size_t> cursors(refs.size(), 0);
    bool has_t0 = false;
    double t0 = 0.0;
    for (const auto& r : recv) {
        int matched = -1;
        for (int i = 0; i < static_cast<int>(refs.size()); ++i) {
            const auto& s = refs[static_cast<std::size_t>(i)];
            if (r.t >= s.front().t && r.t <= s.back().t) {
                matched = i;
                break;
            }
        }
        if (matched < 0) continue;

        RvSample ref;
        const std::size_t idx = static_cast<std::size_t>(matched);
        if (!interpolate_rv_at(refs[idx], r.t, cursors[idx], ref)) continue;

        if (!has_t0) {
            t0 = r.t;
            has_t0 = true;
        }
        const double drx = r.rx - ref.rx;
        const double dry = r.ry - ref.ry;
        const double drz = r.rz - ref.rz;
        const double dvx = r.vx - ref.vx;
        const double dvy = r.vy - ref.vy;
        const double dvz = r.vz - ref.vz;
        out.dt.push_back(r.t - t0);
        out.pos_norm.push_back(std::sqrt(drx * drx + dry * dry + drz * drz));
        out.vel_norm.push_back(std::sqrt(dvx * dvx + dvy * dvy + dvz * dvz));
    }
    return out;
}

static std::vector<double> collect_breakpoints_from_recv(const std::vector<CsvRunRow>& recv_rows) {
    std::vector<double> bp;
    for (const auto& r : recv_rows) {
        if (r.note != "incoming_1") continue;
        if (!std::isfinite(r.t)) continue;
        if (!bp.empty() && std::fabs(bp.back() - r.t) < 1e-9) continue;
        bp.push_back(r.t);
    }
    return bp;
}

static std::vector<Mode1Segment> build_mode1_segments(
    const std::vector<Mode1SheetRows>& mode1_sheets,
    const std::vector<double>& breakpoints) {
    std::vector<Mode1Segment> segments;
    if (mode1_sheets.empty() || breakpoints.empty()) return segments;

    const double eps = 1e-9;
    const size_t n = std::min(mode1_sheets.size(), breakpoints.size());
    segments.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        const auto& sh = mode1_sheets[i];
        const double t0 = breakpoints[i];
        const double t1 = (i + 1 < breakpoints.size())
                            ? breakpoints[i + 1]
                            : std::numeric_limits<double>::infinity();

        Mode1Segment seg;
        seg.idx = sh.idx;
        seg.reuse = sh.reuse;

        for (const auto& r : sh.rows) {
            if (!std::isfinite(r.t)) continue;
            if (r.t + eps < t0) continue;
            if (std::isfinite(t1) && r.t - eps > t1) continue;
            if (!std::isfinite(r.rx) || !std::isfinite(r.ry) || !std::isfinite(r.rz) ||
                !std::isfinite(r.vx) || !std::isfinite(r.vy) || !std::isfinite(r.vz)) {
                continue;
            }

            seg.t.push_back(r.t);
            seg.rx.push_back(r.rx);
            seg.ry.push_back(r.ry);
            seg.rz.push_back(r.rz);
            seg.vx.push_back(r.vx);
            seg.vy.push_back(r.vy);
            seg.vz.push_back(r.vz);
        }

        if (!seg.t.empty()) segments.push_back(std::move(seg));
    }

    return segments;
}

static void ensure_python_interpreter() {
    static std::unique_ptr<py::scoped_interpreter> guard;
    if (!guard) guard = std::make_unique<py::scoped_interpreter>();
}

template <typename Axis3D>
static void apply_equal_3d_limits(
    Axis3D& ax,
    const std::vector<Series3D>& series_list) {
    double xmin = 0.0, xmax = 0.0;
    double ymin = 0.0, ymax = 0.0;
    double zmin = 0.0, zmax = 0.0;
    bool inited = false;

    for (const auto& s : series_list) {
        for (size_t i = 0; i < s.x.size(); ++i) {
            if (!inited) {
                xmin = xmax = s.x[i];
                ymin = ymax = s.y[i];
                zmin = zmax = s.z[i];
                inited = true;
            } else {
                xmin = std::min(xmin, s.x[i]);
                xmax = std::max(xmax, s.x[i]);
                ymin = std::min(ymin, s.y[i]);
                ymax = std::max(ymax, s.y[i]);
                zmin = std::min(zmin, s.z[i]);
                zmax = std::max(zmax, s.z[i]);
            }
        }
    }
    if (!inited) return;

    const double xmid = 0.5 * (xmin + xmax);
    const double ymid = 0.5 * (ymin + ymax);
    const double zmid = 0.5 * (zmin + zmax);
    const double xspan = xmax - xmin;
    const double yspan = ymax - ymin;
    const double zspan = zmax - zmin;
    const double half = 0.5 * std::max({xspan, yspan, zspan, 1e-6});

    ax.set_xlim(Args(xmid - half, xmid + half));
    ax.set_ylim(Args(ymid - half, ymid + half));
    ax.set_zlim(Args(zmid - half, zmid + half));
}

template <typename AxisT>
static void draw_break_lines(
    AxisT& ax,
    const std::vector<double>& breakpoints,
    double ymin,
    double ymax) {
    if (!std::isfinite(ymin) || !std::isfinite(ymax)) {
        ymin = -1.0;
        ymax = 1.0;
    }
    if (std::fabs(ymax - ymin) < 1e-9) {
        ymin -= 1.0;
        ymax += 1.0;
    }
    for (double t : breakpoints) {
        std::vector<double> x{t, t};
        std::vector<double> y{ymin, ymax};
        ax.plot(Args(x, y),
                Kwargs("linestyle"_a = "--", "linewidth"_a = 1.0, "color"_a = "gray", "alpha"_a = 0.6));
    }
}

template <typename AxisT>
static void plot_component_overlay(
    AxisT& ax,
    const std::string& title,
    const SeriesTV& recv,
    const std::vector<Mode1Segment>& segments,
    const std::vector<double>& breakpoints,
    int comp_idx) {
    double ymin = std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();
    auto update_y_bounds = [&](const std::vector<double>& y) {
        for (double v : y) {
            if (!std::isfinite(v)) continue;
            ymin = std::min(ymin, v);
            ymax = std::max(ymax, v);
        }
    };

    if (!recv.t.empty()) {
        ax.plot(Args(recv.t, recv.v),
                Kwargs("label"_a = "recv", "linewidth"_a = 2.0, "color"_a = "black", "alpha"_a = 0.85));
        update_y_bounds(recv.v);
    }

    bool solver_labeled = false;
    bool reuse_labeled = false;
    for (const auto& seg : segments) {
        const std::vector<double>* y = nullptr;
        if (comp_idx == 0) y = &seg.rx;
        if (comp_idx == 1) y = &seg.ry;
        if (comp_idx == 2) y = &seg.rz;
        if (comp_idx == 3) y = &seg.vx;
        if (comp_idx == 4) y = &seg.vy;
        if (comp_idx == 5) y = &seg.vz;
        if (!y || y->empty()) continue;

        const bool reuse = seg.reuse;
        std::string label = "_nolegend_";
        if (reuse && !reuse_labeled) {
            label = "mode1_reuse";
            reuse_labeled = true;
        } else if (!reuse && !solver_labeled) {
            label = "mode1_solver";
            solver_labeled = true;
        }

        ax.plot(Args(seg.t, *y),
                Kwargs("label"_a = label,
                       "linewidth"_a = 1.8,
                       "alpha"_a = 0.9,
                       "color"_a = (reuse ? "tab:orange" : "tab:blue")));
        update_y_bounds(*y);
    }

    draw_break_lines(ax, breakpoints, ymin, ymax);
    ax.set_title(Args(title));
    ax.set_xlabel(Args("t (s)"));
    ax.grid(Args(true));
    ax.legend();
}

bool plot_from_run_csv(const std::string& run_csv_path) {
    const ParsedRunCsv parsed = load_run_csv(run_csv_path);
    if (parsed.sheets.empty()) return false;

    const CsvSheetBlock* recv_sheet = find_sheet(parsed, "receive");
    const CsvSheetBlock* mode0_sheet = find_sheet(parsed, "mode0");
    if (!recv_sheet) return false;
    const std::vector<CsvRunRow>& recv_rows = recv_sheet->rows;
    if (recv_rows.empty()) return false;

    const std::vector<Mode1SheetRows> mode1_sheets = extract_mode1_sheets(parsed);
    if (mode1_sheets.empty()) return false;

    ensure_python_interpreter();
    auto plt = matplotlibcpp17::pyplot::import();

    // Figure 1:
    // 1) R 3D for mode1_1 vs receive
    // 2) receive v_x / v_y / v_z vs t
    auto fig1 = plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(12, 10)));
    auto ax1_3d = fig1.add_subplot(Args(2, 1, 1), Kwargs("projection"_a = "3d"));

    const CsvSheetBlock* mode1_1_sheet = find_sheet(parsed, "mode1_1");
    const Series3D mode1_1_r = mode1_1_sheet ? build_r3d_series(mode1_1_sheet->rows) : Series3D{};
    const Series3D recv_r = build_r3d_series(recv_rows);

    if (!mode1_1_r.x.empty()) {
        ax1_3d.plot(Args(mode1_1_r.x, mode1_1_r.y, mode1_1_r.z),
                    Kwargs("label"_a = "mode1_1", "linewidth"_a = 2.0, "color"_a = "tab:blue"));
    }
    if (!recv_r.x.empty()) {
        ax1_3d.plot(Args(recv_r.x, recv_r.y, recv_r.z),
                    Kwargs("label"_a = "recv", "linewidth"_a = 2.0, "color"_a = "tab:orange"));
    }
    apply_equal_3d_limits(ax1_3d, {mode1_1_r, recv_r});
    ax1_3d.set_xlabel(Args("r_x"));
    ax1_3d.set_ylabel(Args("r_y"));
    ax1_3d.set_zlabel(Args("r_z"));
    ax1_3d.set_title(Args("R 3D: mode1_1 vs recv"));
    ax1_3d.grid(Args(true));
    ax1_3d.legend();

    auto ax1_v = fig1.add_subplot(Args(2, 1, 2));
    const SeriesTV recv_vx = build_tv_series_from_recv(recv_rows, 0);
    const SeriesTV recv_vy = build_tv_series_from_recv(recv_rows, 1);
    const SeriesTV recv_vz = build_tv_series_from_recv(recv_rows, 2);
    if (!recv_vx.t.empty()) {
        ax1_v.plot(Args(recv_vx.t, recv_vx.v), Kwargs("label"_a = "recv_v_x", "linewidth"_a = 1.8, "color"_a = "tab:red"));
    }
    if (!recv_vy.t.empty()) {
        ax1_v.plot(Args(recv_vy.t, recv_vy.v), Kwargs("label"_a = "recv_v_y", "linewidth"_a = 1.8, "color"_a = "tab:green"));
    }
    if (!recv_vz.t.empty()) {
        ax1_v.plot(Args(recv_vz.t, recv_vz.v), Kwargs("label"_a = "recv_v_z", "linewidth"_a = 1.8, "color"_a = "tab:blue"));
    }
    ax1_v.set_title(Args("recv v_x v_y v_z vs t"));
    ax1_v.set_xlabel(Args("t (s)"));
    ax1_v.set_ylabel(Args("v"));
    ax1_v.grid(Args(true));
    ax1_v.legend();

    // Figure 2:
    // Stitch/overlay all mode1 sheets by incoming_1 breakpoints and compare with recv in 6 subplots.
    const std::vector<double> breakpoints = collect_breakpoints_from_recv(recv_rows);
    const std::vector<Mode1Segment> segments = build_mode1_segments(mode1_sheets, breakpoints);

    auto fig2 = plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(16, 11)));
    auto ax_rx = fig2.add_subplot(Args(3, 2, 1));
    auto ax_ry = fig2.add_subplot(Args(3, 2, 2));
    auto ax_rz = fig2.add_subplot(Args(3, 2, 3));
    auto ax_vx = fig2.add_subplot(Args(3, 2, 4));
    auto ax_vy = fig2.add_subplot(Args(3, 2, 5));
    auto ax_vz = fig2.add_subplot(Args(3, 2, 6));

    const SeriesTV recv_rx = build_tv_series_from_recv(recv_rows, 3);
    const SeriesTV recv_ry = build_tv_series_from_recv(recv_rows, 4);
    const SeriesTV recv_rz = build_tv_series_from_recv(recv_rows, 5);

    plot_component_overlay(ax_rx, "r_x vs t", recv_rx, segments, breakpoints, 0);
    plot_component_overlay(ax_ry, "r_y vs t", recv_ry, segments, breakpoints, 1);
    plot_component_overlay(ax_rz, "r_z vs t", recv_rz, segments, breakpoints, 2);
    plot_component_overlay(ax_vx, "v_x vs t", recv_vx, segments, breakpoints, 3);
    plot_component_overlay(ax_vy, "v_y vs t", recv_vy, segments, breakpoints, 4);
    plot_component_overlay(ax_vz, "v_z vs t", recv_vz, segments, breakpoints, 5);

    // Figure 3:
    // New window for per-dt norm distance between recv and mode0.
    {
        ErrorNormSeries err = build_recv_segment_error_norm_series(recv_rows, segments);
        std::string ref_name = "stitched_mode1";
        if (err.dt.empty() && mode0_sheet && !mode0_sheet->rows.empty()) {
            err = build_recv_mode0_error_norm_series(recv_rows, mode0_sheet->rows);
            ref_name = "mode0";
        }
        if (!err.dt.empty()) {
            const std::string pos_label = "||recv_r - " + ref_name + "_r||";
            const std::string vel_label = "||recv_v - " + ref_name + "_v||";
            auto fig3 = plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(12, 8)));
            auto ax_pos = fig3.add_subplot(Args(2, 1, 1));
            auto ax_vel = fig3.add_subplot(Args(2, 1, 2));

            ax_pos.plot(
                Args(err.dt, err.pos_norm),
                Kwargs("label"_a = pos_label, "linewidth"_a = 2.0, "color"_a = "tab:purple"));
            ax_pos.set_title(Args("Position Distance Error vs dt (clipped reference)"));
            ax_pos.set_xlabel(Args("dt from first recv sample (s)"));
            ax_pos.set_ylabel(Args("distance (m)"));
            ax_pos.grid(Args(true));
            ax_pos.legend();

            ax_vel.plot(
                Args(err.dt, err.vel_norm),
                Kwargs("label"_a = vel_label, "linewidth"_a = 2.0, "color"_a = "tab:brown"));
            ax_vel.set_title(Args("Velocity Distance Error vs dt (clipped reference)"));
            ax_vel.set_xlabel(Args("dt from first recv sample (s)"));
            ax_vel.set_ylabel(Args("distance (m/s)"));
            ax_vel.grid(Args(true));
            ax_vel.legend();
        }
    }

    plt.show();
    return true;
}

bool debug_cfg_from_run_csv(
    const std::string& run_csv_path,
    double target_elapsed_sec,
    std::string& mode1_sheet_name_out,
    std::string& cfg_out) {
    mode1_sheet_name_out.clear();
    cfg_out.clear();

    const ParsedRunCsv parsed = load_run_csv(run_csv_path);
    if (parsed.sheets.empty()) return false;

    const CsvSheetBlock* recv_sheet = find_sheet(parsed, "receive");
    if (!recv_sheet || recv_sheet->rows.empty()) return false;

    std::vector<double> bp = collect_breakpoints_from_recv(recv_sheet->rows);
    if (bp.empty()) return false;

    int best_idx = 0;
    double best_abs_dt = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(bp.size()); ++i) {
        const double dt = std::fabs(bp[i] - target_elapsed_sec);
        if (dt < best_abs_dt) {
            best_abs_dt = dt;
            best_idx = i;
        }
    }

    const int mode1_idx = best_idx + 1;
    mode1_sheet_name_out = "mode1_" + std::to_string(mode1_idx);
    const CsvSheetBlock* mode1_sheet = find_sheet(parsed, mode1_sheet_name_out);
    if (!mode1_sheet || mode1_sheet->rows.empty()) return false;

    for (const auto& r : mode1_sheet->rows) {
        if (!r.cfg.empty()) {
            cfg_out = r.cfg;
            return true;
        }
    }
    return false;
}
