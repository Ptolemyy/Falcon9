# Falcon 9 GUI Planner

This project is a Windows C++ GUI mission planner for:

- Launch ascent (simplified 3DOF-inspired propagation)
- Stage-1 separation and sea-recovery budget
- Stage-2 orbit insertion budget

You can set orbit parameters (perigee, apogee, inclination) in the UI.
Vehicle/stage/payload/launch parameters are loaded from `send.txt` config.

## Features

- Parameter input panel for orbit settings only
- Vehicle config can be loaded from a `send.txt` file via `--vehicle-config`
- `Plan Mission` button to run trajectory and budget estimation
- `Compute Max Payload` button to estimate max feasible payload at current orbit settings
- Mission report list with timeline and feasibility checks
- Stage-1 separation is auto-optimized (no manual sep altitude/speed tuning required)
- Droneship downrange is auto-optimized by planner (input value used as initial guess)
- Stage-1 ascent guidance uses an UPFG-like terminal-state guidance law (simplified 2D implementation)
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
lat_deg=28.5
launch_lon_deg=-80.6
ship_downrange_km=620
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
