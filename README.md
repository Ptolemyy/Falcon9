# Falcon 9 GUI Planner

This project is a Windows C++ GUI mission planner for:

- Launch ascent (event-based C++ LVD-style stage-1 reference propagation)
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
- Stage-1 LVD is solved once per payload with a mission-scored direct search over SEP targets plus steering/throttle node parameters, then separation time is searched by clipping the LVD reference trajectory
- Droneship downrange is set to Stage-1 ballistic impact prediction (with atmospheric drag)
- Stage-1 ascent uses a C++ LVD-style event model with launch-window/planet-rotation inputs
- Stage-2 orbit insertion uses propagated UPFG-like guidance instead of a Bezier approximation
- Single-window DISPLAY view:
  - Blue Marble Earth texture with atmosphere-style rim
  - Mission trajectories projected around Earth
  - Post-insertion Stage-2 orbit track around Earth
  - Launch site and droneship markers
- Mouse drag rotation of globe view
- In-window sweep charts for Stage-2 remaining fuel, landing margin, and landing propellant
- In-window LVD events and launch-window diagnostics

- Separate communication bridge executable:
  - waits for `send.txt`
  - validates key/value payload
  - launches planner with `--vehicle-config <send.txt>`

## Build

Requirements:

- Windows
- CMake >= 3.16
- Visual Studio Build Tools or Visual Studio C++ toolchain
- Windows SDK
- Ninja is recommended for the current Clang build

Current Clang/Ninja build used in this workspace:

```powershell
cmake -S . -B build -G Ninja `
  -DCMAKE_BUILD_TYPE=Debug `
  -DCMAKE_CXX_COMPILER="C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Tools/Llvm/x64/bin/clang.exe"

cmake --build build --target falcon9_gui_planner
```

Release Clang/Ninja build in a clean directory:

```powershell
cmake -S . -B build-clang-release -G Ninja `
  -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_CXX_COMPILER="C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Tools/Llvm/x64/bin/clang.exe"

cmake --build build-clang-release
```

Visual Studio generator still works, but it is multi-config and writes executables under the selected configuration folder:

```powershell
cmake -S . -B build-vs -G "Visual Studio 17 2022" -A x64
cmake --build build-vs --config Release
```

Executable locations:

- Ninja single-config: `build/falcon9_gui_planner.exe`
- Ninja release example: `build-clang-release/falcon9_gui_planner.exe`
- Visual Studio multi-config: `build-vs/Release/falcon9_gui_planner.exe`

During configure, CMake copies:

- `config/falcon9_real_defaults.txt` -> `<build-dir>/falcon9_real_defaults.txt`
- `assets/earth_blue_marble.bmp` -> `<build-dir>/assets/earth_blue_marble.bmp`

## Run

```powershell
.\build\falcon9_gui_planner.exe
```

When started without `--vehicle-config`, the planner will try to auto-load:

`<build-dir>/falcon9_real_defaults.txt` (copied from `config/falcon9_real_defaults.txt` during configure)

Load vehicle config from file:

```powershell
.\build\falcon9_gui_planner.exe --vehicle-config .\send.txt
```

Run communication bridge (separate cpp):

```powershell
.\build\planner_comm_bridge.exe .\send.txt .\build\falcon9_gui_planner.exe 120
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
launch_utc=2026-05-01T00:00:00Z
# earth_rotation_angle_deg=0
launch_window_half_width_min=45
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
  
