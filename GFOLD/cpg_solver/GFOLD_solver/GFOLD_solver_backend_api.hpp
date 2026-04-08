#pragma once

#include <cstddef>

struct GFOLDBackendConfig {
    int steps = 0;
    double tf = 0.0;
    double elapsed_time = 0.0;
    double g0 = 0.0;
    double Isp = 0.0;
    double T_max = 0.0;
    double throttle_min = 0.0;
    double throttle_max = 0.0;
    double m0 = 0.0;
    double r0[3] = {0.0, 0.0, 0.0};
    double v0[3] = {0.0, 0.0, 0.0};
    double glide_slope_deg = 0.0;
    double cot_y_gs = 0.0;
    double max_angle_deg = 0.0;
};

struct GFOLDBackendInfo {
    int status = -999;
    int iter = 0;
    double obj_val = 0.0;
    double pri_res = 0.0;
    double dua_res = 0.0;
};

struct GFOLDBackendLimits {
    double feastol = 0.0;
    double abstol = 0.0;
    double reltol = 0.0;
    double feastol_inacc = 0.0;
    double abstol_inacc = 0.0;
    double reltol_inacc = 0.0;
    int maxit = 0;
};

struct GFOLDBackendOutput {
    int steps = 0;
    std::size_t capacity = 0;

    // Caller-provided buffers with at least `capacity` doubles each.
    double* t = nullptr;
    double* ux = nullptr;
    double* uy = nullptr;
    double* uz = nullptr;
    double* vx = nullptr;
    double* vy = nullptr;
    double* vz = nullptr;
    double* rx = nullptr;
    double* ry = nullptr;
    double* rz = nullptr;
    double* z = nullptr;

    GFOLDBackendInfo info{};
    GFOLDBackendLimits limits{};
    double terminal_mass = 0.0;
};
