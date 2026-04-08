#pragma once

#include "GFOLD_solver.hpp"
#include <vector>

struct SearchResult {
    double best_tf = 0.0;
    double best_m  = -1.0;
    std::vector<double> last_t;
    std::vector<double> last_ux;
    std::vector<double> last_uy;
    std::vector<double> last_uz;
    std::vector<double> last_rx;
    std::vector<double> last_ry;
    std::vector<double> last_rz;
    std::vector<double> last_vx;
    std::vector<double> last_vy;
    std::vector<double> last_vz;
    std::vector<double> last_m_traj;
    int last_steps = 0;
    double last_tf = 0.0;
    bool has_last_traj = false;
    double elapsed_sec = 0.0;
    long long solve_calls = 0;
    bool feasible = false;
};

struct ThrottleSearchResult {
    double best_throttle_max = 1.0;
    double elapsed_sec = 0.0;
    long long solve_calls = 0;
    bool feasible = false;
};

// Search for best tf in [a, b] (seconds) maximizing terminal mass.
// If save_last_traj is true, store full u/r/v/m trajectory from the last evaluated feasible solve.
SearchResult find_best_tf(const GFOLDConfig& cfg, double a, double b, int iters = 20, bool save_last_traj = false);

// Search minimum feasible throttle_max in (throttle_lo, throttle_hi].
ThrottleSearchResult find_min_feasible_throttle_max(
    const GFOLDConfig& cfg,
    double throttle_lo,
    double throttle_hi = 1.0,
    int iters = 20);
