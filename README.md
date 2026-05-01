# Falcon 9 GUI Planner

This project is a Windows C++ GUI mission planner for:

- Launch ascent (simplified 3DOF-inspired propagation)
- Stage-1 separation and sea-recovery budget
- Stage-2 orbit insertion budget

You can set orbit parameters (perigee, apogee, inclination) in the UI.
Vehicle/stage/payload/launch parameters are loaded from `send.txt` config.

## Features

- Parameter input panel for orbit settings only
- Vehicle config can be loaded from `--vehicle-config <file>`
- Built-in default config (`config/falcon9_real_defaults.txt`) is auto-loaded at startup when no config argument is given
- `Plan Mission` button to run trajectory and budget estimation
- `Plan No-Recovery Burnout` button to ignore recovery and separate after Stage-1 propellant depletion
- `Compute Max Payload` button to estimate max feasible payload at current orbit settings
- `Import Default Falcon9 Config` button to load default vehicle values from the UI
- `<` / `>` candidate buttons to inspect stored separation-time candidates in the trajectory and globe views
- Mission report list with timeline and feasibility checks
- Stage-1 separation is auto-optimized (no manual sep altitude/speed tuning required)
- Droneship downrange is set to Stage-1 ballistic impact prediction (with atmospheric drag)
- Stage-1 ascent guidance uses a constrained UPFG-like Falcon 9 / Starlink-style ascent law (simplified 2D implementation)
- Stage-2 orbit insertion uses propagated UPFG-like guidance instead of a Bezier approximation
- 2D trajectory profile view:
  - Stage-1 ascent
  - Stage-1 sea recovery path
  - Stage-2 orbit injection path
- 3D whole-Earth view (orthographic projection):
  - Earth sphere with latitude/longitude grid
  - Mission trajectories projected around Earth
  - Post-insertion Stage-2 orbit track around Earth
  - Launch site and droneship markers
- Mouse drag rotation of globe view
- Separate sweep charts window for Stage-2 remaining fuel, landing margin, and landing propellant

- Separate communication bridge executable:
  - waits for `send.txt`
  - validates key/value payload
  - launches planner with `--vehicle-config <send.txt>`

## Build

Requirements:

- Windows
- CMake >= 3.16
- Visual Studio C++ toolchain

Commands (from repository root):

```powershell
cmake -S . -B build
cmake --build build --config Release
```

Executable:

`build/Release/falcon9_gui_planner.exe`

`build/Release/planner_comm_bridge.exe`

## Run

```powershell
.\build\Release\falcon9_gui_planner.exe
```

When started without `--vehicle-config`, the planner will try to auto-load:

`build/Release/falcon9_real_defaults.txt` (copied from `config/falcon9_real_defaults.txt` during build)

Load vehicle config from file:

```powershell
.\build\Release\falcon9_gui_planner.exe --vehicle-config .\send.txt
```

Run communication bridge (separate cpp):

```powershell
.\build\Release\planner_comm_bridge.exe .\send.txt .\build\Release\falcon9_gui_planner.exe 120
```

- arg1: send file path
- arg2: planner executable path
- arg3: wait timeout seconds

## send.txt Format

The planner accepts numeric `key=value` lines. Example:

```text
payload_kg=15000
s1_dry_kg=25600
s1_prop_kg=395700
s1_isp_s=282
s1_thrust_kN=7607
s2_dry_kg=4000
s2_prop_kg=92670
s2_isp_s=348
s2_thrust_kN=981
s2_ignition_delay_s=7
s2_target_seco_s=529
lat_deg=28.5
launch_lon_deg=-80.6
ship_downrange_km=620
q_limit_kpa=45
s1_target_maxq_kpa=35
s1_target_meco_s=145
s1_sep_delay_s=3
```

Orbit UI fields remain editable in planner (`Perigee`, `Apogee`, `Inclination`).

## kOS Script Notes

- Script file: `script/kos_planner_main.ks`
- Engine/part tags are optional.
- If tags are absent, script auto-splits engines (highest-Isp engine -> stage2) and estimates missing stage1 prop from current stack mass with safe defaults.

## Notes on the Model

- This is a planning/visualization tool, not a high-fidelity flight dynamics simulator.
- Ascent and recovery logic uses simplified assumptions (density model, drag treatment, pitch schedule, and budget heuristics).
- Stage-1 recovery and Stage-2 insertion outputs should be treated as engineering estimates for rapid iteration.

## Main Files

- `falcon9_gui_planner.cpp` - GUI, planning logic, 2D/3D rendering
- `planner_comm_bridge.cpp` - standalone send.txt bridge/launcher
- `script/kos_planner_main.ks` - kOS sender script template
- `CMakeLists.txt` - root build configuration
  
