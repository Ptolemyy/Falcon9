#pragma once

#include "planner_mission.hpp"

namespace falcon9 {

RecoveryResult simulate_stage1_recovery(
    const MissionRequest& request,
    const Stage1Result& stage1,
    double launch_az_deg);

}  // namespace falcon9
