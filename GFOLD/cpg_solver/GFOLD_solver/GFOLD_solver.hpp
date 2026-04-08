// Simple wrapper around the CVXPYgen-generated cpg_* API so callers can
// configure the landing problem in one place and run the solver.
#pragma once

#include <limits>
#include <vector>

struct GFOLDConfig {
    // Discretization
    int steps = 100;           // number of shooting nodes (must match generated model)
    double tf = 57.29;         // final time (s)
    double elapsed_time = 0.0; // absolute elapsed time for node 0 (s)
    int solver_n = 100;        // backend route key (3, 10, 25, 50, 100)

    // Vehicle / environment
    double g0 = 3.71;          // gravity magnitude (m/s^2)
    double Isp = 2000.0;       // specific impulse (s)
    double T_max = 24000.0;    // maximum thrust (N)
    double throttle_min = 0.2; // minimum throttle fraction
    double throttle_max = 0.8; // maximum throttle fraction
    double m0 = 2000.0;        // initial mass (kg)

    // Initial state
    double r0[3] = {2400.0, 450.0, -330.0};   // x, y, z (m)
    double v0[3] = {-10.0, -40.0, 10.0};      // vx, vy, vz (m/s)

    // Constraints
    double glide_slope_deg = 30.0; // glide slope angle (deg)
    double cot_y_gs = std::numeric_limits<double>::quiet_NaN(); // optional cot(glide_slope)
    double max_angle_deg   = 45.0; // thrust cone half-angle (deg)
};

struct GFOLDThrustProfile {
    std::vector<double> t;          // time stamps (s)
    std::vector<double> thrust_N;   // thrust magnitude (N)
    std::vector<double> angle_deg;  // angle wrt x-axis (deg)
};

struct GFOLDSolverInfo {
    int status = -999;
    int iter = 0;
    double obj_val = std::numeric_limits<double>::quiet_NaN();
    double pri_res = std::numeric_limits<double>::quiet_NaN();
    double dua_res = std::numeric_limits<double>::quiet_NaN();
};

struct GFOLDSolverLimits {
    double feastol = std::numeric_limits<double>::quiet_NaN();
    double abstol = std::numeric_limits<double>::quiet_NaN();
    double reltol = std::numeric_limits<double>::quiet_NaN();
    double feastol_inacc = std::numeric_limits<double>::quiet_NaN();
    double abstol_inacc = std::numeric_limits<double>::quiet_NaN();
    double reltol_inacc = std::numeric_limits<double>::quiet_NaN();
    int maxit = 0;
};

struct GFOLDSolution {
    int steps = 0;
    std::vector<double> t; // absolute elapsed time per node (s)
    std::vector<double> ux;
    std::vector<double> uy;
    std::vector<double> uz;
    std::vector<double> vx;
    std::vector<double> vy;
    std::vector<double> vz;
    std::vector<double> rx;
    std::vector<double> ry;
    std::vector<double> rz;
    std::vector<double> z;
};

class GFOLDSolver {
public:
    explicit GFOLDSolver(const GFOLDConfig& cfg = {});

    void set_config(const GFOLDConfig& cfg);
    const GFOLDConfig& config() const { return cfg_; }

    // Route by provided backend n for this solve call.
    bool solve(int solver_n);
    bool solve();
    int status() const;
    GFOLDSolverInfo info() const;
    GFOLDSolverLimits limits() const;
    GFOLDSolution solution() const;
    double terminal_mass() const;

    GFOLDThrustProfile compute_thrust_profile() const;

private:
    bool solve_with_backend_n(int solver_n);

    GFOLDConfig cfg_;
    GFOLDSolution last_solution_;
    GFOLDSolverInfo last_info_;
    GFOLDSolverLimits last_limits_;
    double last_terminal_mass_ = std::numeric_limits<double>::quiet_NaN();
};
