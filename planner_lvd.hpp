#pragma once

#include "planner_mission.hpp"

#include <functional>

namespace falcon9 {

struct LvdResult;

struct LvdOptions {
    bool allow_full_burn = false;
    bool force_stage1_burnout = false;
    SteeringModel3D steering_model;
    ThrottleModel throttle_model;
    const std::atomic<bool>* cancel_requested = nullptr;
    std::function<double(const LvdResult&)> mission_score;
    int max_mission_design_evals = 0;
};

struct LvdResult {
    Stage1Result stage1;
    std::vector<LvdStateSample> state_samples;
    std::vector<Series> time_series;
    std::vector<LvdEvent> events;
    std::vector<LaunchWindowSample> launch_window_samples;
    double target_sep_alt_km = 0.0;
    double target_sep_speed_mps = 0.0;
    double target_sep_gamma_deg = 0.0;
    double launch_offset_s = 0.0;
    double earth_rotation_angle_deg = 0.0;
    double launch_raan_deg = 0.0;
    double target_raan_deg = std::numeric_limits<double>::quiet_NaN();
    double plane_error_deg = 0.0;
};

LvdResult solve_stage1_lvd(
    const MissionRequest& request,
    const OrbitTarget& orbit_target,
    const LvdOptions& options = {});

}  // namespace falcon9
