#pragma once

#include "planner_mission.hpp"

namespace falcon9 {

struct UpfgVehicle {
    double thrust_N = 0.0;
    double isp_s = 0.0;
    double min_throttle = 1.0;
    double max_throttle = 1.0;
};

struct UpfgTarget {
    double target_r_m = kRe;
    double target_vr_mps = 0.0;
    double target_vt_mps = 0.0;
};

struct UpfgSettings {
    double terminal_time_s = 6.0;
    double gamma_min_deg = -8.0;
    double gamma_max_deg = 35.0;
    double radial_gain = 1.25;
    double tangential_gain = 1.20;
    double radial_position_gain = 1.10;
    double max_radial_accel = 8.0;
    double max_tangential_accel = 12.0;
};

struct UpfgCommand {
    double tgo_s = 0.0;
    double vgo_mps = 0.0;
    double gamma_cmd_rad = 0.0;
    double throttle = 1.0;
    bool converged = false;
};

void propagate_polar_coast(PolarState& s, double dt);
UpfgCommand upfg_compute_command(
    const PolarState& state,
    const UpfgVehicle& vehicle,
    const UpfgTarget& target,
    const UpfgSettings& settings,
    double prev_tgo_s,
    double dt);

}  // namespace falcon9
