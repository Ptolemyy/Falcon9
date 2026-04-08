#include "find_cfg.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

SearchResult find_best_tf(const GFOLDConfig& cfg_in, double a, double b, int iters, bool save_last_traj) {
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();

    SearchResult res;

    GFOLDSolver solver(cfg_in);
    const int steps = cfg_in.steps;

    auto capture_last_traj = [&](double tf) {
        GFOLDSolution sol = solver.solution();
        if (sol.steps != steps ||
            static_cast<int>(sol.t.size()) != steps ||
            static_cast<int>(sol.ux.size()) != steps ||
            static_cast<int>(sol.uy.size()) != steps ||
            static_cast<int>(sol.uz.size()) != steps ||
            static_cast<int>(sol.rx.size()) != steps ||
            static_cast<int>(sol.ry.size()) != steps ||
            static_cast<int>(sol.rz.size()) != steps ||
            static_cast<int>(sol.vx.size()) != steps ||
            static_cast<int>(sol.vy.size()) != steps ||
            static_cast<int>(sol.vz.size()) != steps ||
            static_cast<int>(sol.z.size()) != steps) {
            return;
        }

        res.last_ux = std::move(sol.ux);
        res.last_uy = std::move(sol.uy);
        res.last_uz = std::move(sol.uz);
        res.last_t  = std::move(sol.t);
        res.last_rx = std::move(sol.rx);
        res.last_ry = std::move(sol.ry);
        res.last_rz = std::move(sol.rz);
        res.last_vx = std::move(sol.vx);
        res.last_vy = std::move(sol.vy);
        res.last_vz = std::move(sol.vz);
        res.last_m_traj.resize(steps);
        for (int i = 0; i < steps; ++i) {
            res.last_m_traj[i] = std::exp(sol.z[i]);
        }
        res.last_steps = steps;
        res.last_tf = tf;
        res.has_last_traj = true;
    };

    auto f = [&](double x) -> double {
        res.solve_calls++;
        GFOLDConfig trial = cfg_in;
        trial.tf = x;
        trial.throttle_max = cfg_in.throttle_max;
        trial.glide_slope_deg = cfg_in.glide_slope_deg;
        solver.set_config(trial);
        if (!solver.solve()) return -std::numeric_limits<double>::infinity(); // infeasible

        const double mass = solver.terminal_mass();
        if (mass != mass) return -std::numeric_limits<double>::infinity();
        if (save_last_traj) {
            // Overwrite with the latest feasible iterate trajectory.
            capture_last_traj(x);
        }

        res.feasible = true;
        if (mass > res.best_m) { res.best_m = mass; res.best_tf = x; }
        return mass;
    };

    constexpr double CGOLD = 0.3819660112501051; // 1 - 1/phi
    constexpr double eps   = 1e-12;

    double x = a + 0.5 * (b - a);
    double w = x, v = x;
    double fx = f(x), fw = fx, fv = fx;

    double d = 0.0, e = 0.0;

    for (int iter = 0; iter < iters; ++iter) {
        const double m  = 0.5 * (a + b);
        const double tol = std::sqrt(eps) * (std::fabs(x) + 1.0);

        if (std::fabs(x - m) <= 2.0 * tol - 0.5 * (b - a)) break;

        double p = 0.0, q = 0.0, r = 0.0;
        bool do_parabola = (std::fabs(e) > tol) &&
                           (fx > -std::numeric_limits<double>::infinity()) &&
                           (fw > -std::numeric_limits<double>::infinity()) &&
                           (fv > -std::numeric_limits<double>::infinity());

        if (do_parabola) {
            r = (x - w) * (fx - fv);
            q = (x - v) * (fx - fw);
            p = (x - v) * q - (x - w) * r;
            q = 2.0 * (q - r);
            if (q > 0.0) p = -p;
            q = std::fabs(q);

            const double etemp = e;
            e = d;

            if (std::fabs(p) < std::fabs(0.5 * q * etemp) &&
                p > q * (a - x) &&
                p < q * (b - x)) {
                d = p / q;
            } else {
                e = (x < m) ? (b - x) : (a - x);
                d = CGOLD * e;
            }
        } else {
            e = (x < m) ? (b - x) : (a - x);
            d = CGOLD * e;
        }

        double u = x + ((std::fabs(d) >= tol) ? d : (d > 0 ? tol : -tol));
        double fu = f(u);

        if (fu >= fx) {
            if (u >= x) a = x; else b = x;
            v = w; fv = fw;
            w = x; fw = fx;
            x = u; fx = fu;
        } else {
            if (u < x) a = u; else b = u;
            if (fu >= fw) {
                v = w; fv = fw;
                w = u; fw = fu;
            } else if (fu >= fv) {
                v = u; fv = fu;
            }
        }
    }

    auto t1 = clock::now();
    res.elapsed_sec = std::chrono::duration<double>(t1 - t0).count();
    return res;
}

ThrottleSearchResult find_min_feasible_throttle_max(
    const GFOLDConfig& cfg_in,
    double throttle_lo,
    double throttle_hi,
    int iters) {
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();

    ThrottleSearchResult res;
    constexpr double eps = 1e-9;

    double lo = std::max(0.0, throttle_lo);
    double hi = std::min(1.0, throttle_hi);
    if (hi <= lo + eps) {
        auto t1 = clock::now();
        res.elapsed_sec = std::chrono::duration<double>(t1 - t0).count();
        return res;
    }

    GFOLDSolver solver(cfg_in);
    auto is_feasible = [&](double throttle_max) -> bool {
        res.solve_calls++;
        GFOLDConfig trial = cfg_in;
        trial.throttle_max = throttle_max;
        solver.set_config(trial);
        return solver.solve();
    };

    if (!is_feasible(hi)) {
        auto t1 = clock::now();
        res.elapsed_sec = std::chrono::duration<double>(t1 - t0).count();
        return res;
    }

    res.feasible = true;
    res.best_throttle_max = hi;
    for (int i = 0; i < iters; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (mid <= lo + eps || hi <= mid + eps) break;

        if (is_feasible(mid)) {
            res.best_throttle_max = mid;
            hi = mid;
        } else {
            lo = mid;
        }
    }

    auto t1 = clock::now();
    res.elapsed_sec = std::chrono::duration<double>(t1 - t0).count();
    return res;
}
