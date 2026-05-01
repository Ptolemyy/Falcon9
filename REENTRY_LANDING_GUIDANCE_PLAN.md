# Reentry Burn And Landing Burn Guidance Plan

## 1. Purpose

This document defines a planning-period guidance architecture for Stage-1 recovery after separation.
It is intended to serve two use cases with the same algorithm contract:

1. The current Falcon 9 planner, using 2D propagation and fast mission evaluation.
2. A future real-time guidance stack, using the same planning modules with higher-fidelity state estimation and onboard execution.

The design goal is:

1. Keep the optimizer structure compatible with `CVXPYGEN`.
2. Use the existing `GFOLD` stack for landing burn planning first.
3. Make reentry burn planning a small online optimizer with explicit mission tradeoffs.
4. Keep the full recovery chain usable both for offline planning and future real guidance.

This is a planning document, not yet an implementation patch.

## 2. Scope

This document covers only:

1. Stage-1 reentry burn planning.
2. Stage-1 landing burn planning.
3. The interface between them.
4. The shared 2D state, propagation, and solver contracts.

This document does not yet define:

1. Grid fin guidance.
2. Bank / heading control.
3. Full 3DOF or 6DOF aero guidance.
4. Ocean-platform touchdown point assignment.
5. Engine-out handling.
6. Sensor fusion details.

## 3. Existing Local Assets To Reuse

The current repo already contains the building blocks we should keep aligned with:

1. `GFOLD` C++ wrapper and generated backends:
   `GFOLD/cpg_solver/GFOLD_solver/GFOLD_solver.hpp`
2. Existing generated `p4_n50` variant:
   `GFOLD/cpg_solver/variants/p4_n50/CMakeLists.txt`
3. Existing `CVXPYGEN` generation path:
   `GFOLD/solver_py/solver.py`

The `GFOLD` wrapper already exposes:

1. `GFOLDConfig`
2. `GFOLDSolver`
3. `GFOLDSolution`
4. Multiple generated backends by node count

That means the preferred implementation path is:

1. Keep the landing burn planner inside the `GFOLD + CVXPYGEN` ecosystem.
2. Add a dedicated new variant for the free-touchdown-position 2D landing problem.
3. Keep reentry burn planning on a `CVXPYGEN`-compatible convex core plus a small outer search loop.

## 4. Shared Recovery Architecture

The recovery chain is split into four modules:

1. `RecoveryPropagation2D`
2. `ReentryBurnPlanner`
3. `PostEntryPropagation2D`
4. `LandingBurnPlannerGFOLD`

Data flow:

1. Start from stage-separation state from ascent.
2. Propagate ballistic / aero motion forward until candidate reentry-burn ignition times.
3. Solve a small online optimization for reentry-burn timing and target post-burn state.
4. Propagate the selected post-burn state down to the landing-burn handoff region.
5. Solve landing burn using a 2D `GFOLD P4 N=50` variant with free downrange terminal position.
6. Return the full planned recovery trajectory, fuel split, and feasibility report.

The same interfaces are used later for real guidance:

1. The planner version runs from simulated state.
2. The real guidance version runs from filtered onboard state.
3. Both use the same planning modules and parameter names.

## 5. Shared 2D State Definition

All planning-period recovery logic uses a single vertical plane.

State vector:

```text
x = [h, s, vh, vs, m]
```

Where:

1. `h`: altitude above landing surface, meters
2. `s`: downrange position along the recovery plane, meters
3. `vh`: vertical velocity, meters per second
4. `vs`: downrange velocity, meters per second
5. `m`: vehicle mass, kilograms

Control vector for powered phases:

```text
u = [Th, Ts]
```

Where:

1. `Th`: vertical thrust component
2. `Ts`: downrange thrust component

Derived environment values used everywhere:

1. `rho(h)`: atmosphere density
2. `q = 0.5 * rho * v^2`
3. `v = sqrt(vh^2 + vs^2)`
4. Heat surrogate `H = rho^0.5 * v^3`
5. `g(h)`: gravity
6. Drag vector from `CdA`

The current planner and future guidance must use the same state names even if the propagation fidelity changes.

## 6. Coordinate Mapping To Existing GFOLD

The existing `GFOLD` wrapper uses a 3-axis state, but the first axis is the vertical axis in practice.
For the 2D landing-burn planner we embed the 2D state into the existing 3D API as:

```text
r = [h, s, 0]
v = [vh, vs, 0]
u = [uh, us, 0]
```

Mapping rules:

1. `r[0] = h`
2. `r[1] = s`
3. `r[2] = 0`
4. `v[0] = vh`
5. `v[1] = vs`
6. `v[2] = 0`
7. Gravity points along negative `r[0]`

This keeps compatibility with the current `GFOLDConfig` interface while making the landing planner explicitly 2D.

## 7. Landing Burn Planner, Planning-Version v1

### 7.1 Solver Choice

Use a dedicated `GFOLD` backend derived from the existing `p4_n50` solver path:

1. Base family: `P4`
2. Nodes: `N = 50`
3. Dimension: 2D embedded in the current 3D wrapper
4. Code generation path: continue using `CVXPYGEN`

Recommended new backend name:

```text
GFOLD p4_n50_2d_free_x
```

Meaning:

1. `p4`: stay on the current `GFOLD` problem family path already present in the repo
2. `n50`: fixed 50-node horizon
3. `2d`: only altitude + downrange
4. `free_x`: free terminal downrange position

### 7.2 Planning-Version Terminal Conditions

The planning-version landing burn must not constrain landing point position.
It only constrains terminal height to zero, while still enforcing a soft touchdown state.

Terminal constraints:

1. `h_N = 0`
2. `vh_N = 0`
3. `vs_N = 0`
4. `s_N` is free
5. Cross-plane states remain zero because this is 2D

Important clarification:

1. "Do not constrain landing point position" means `s_N` is not fixed to a target.
2. We still require touchdown altitude to be zero.
3. We still require terminal velocities to be near zero because this is a landing burn planner, not a fly-through planner.

### 7.3 Objective

The planning-version landing burn objective is:

1. Primary: minimize total propellant usage
2. Secondary: optional very small numerical regularization on `s_N`

Recommended objective:

```text
min sum(sigma_k) + eps * s_N^2
```

With:

1. `eps` small enough to avoid acting as a landing-point target
2. The regularizer used only to prevent arbitrary drift in otherwise equivalent solutions

This regularization is not a mission landing-site constraint.

### 7.4 Constraints

Keep the existing `GFOLD` convexification structure wherever possible:

1. 2D translational dynamics in the embedded 3D state
2. Log-mass dynamics
3. Thrust magnitude bounds
4. Thrust pointing cone
5. Glide-slope / descent cone
6. Velocity bounds

Planning-version constraints to keep:

1. `N = 50`
2. Fixed or searched terminal time `tf`
3. Vehicle mass, Isp, thrust, throttle limits
4. Glide-slope and thrust cone
5. Terminal `h_N = 0`
6. Terminal `vh_N = 0`
7. Terminal `vs_N = 0`

Planning-version constraints to remove:

1. Any hard terminal downrange equality
2. Any hard terminal lateral position target

### 7.5 Landing Burn Inputs

The landing planner receives:

1. Handoff state `x_lb0 = [h, s, vh, vs, m]`
2. Vehicle parameters:
   `Isp, T_max, throttle_min, throttle_max`
3. Gravity and atmosphere model used by the planner
4. Solver settings:
   `N = 50`, `tf search range`, `solver_n = 50`

### 7.6 Landing Burn Outputs

The landing planner returns:

1. `feasible`
2. `tf`
3. `terminal_mass`
4. `GFOLDSolution`
5. `GFOLDThrustProfile`
6. Predicted terminal `s_N`
7. Predicted touchdown time
8. Feasibility margins and solver status

### 7.7 Landing Burn Use In Current Planner

The planner version uses the landing solver in two roles:

1. Final landing-burn trajectory generation
2. Feasibility oracle for the reentry-burn optimizer

For the current planner:

1. The descent arc is propagated in 2D.
2. Candidate handoff states are checked against the landing solver.
3. The earliest or lowest-fuel feasible handoff can be selected depending on mode.

### 7.8 Landing Burn Use In Future Real Guidance

For real guidance later:

1. Replan from filtered onboard state at a lower rate than the inner control loop.
2. Use the same `GFOLD` interface.
3. Track the returned thrust direction / throttle profile with the real engine controller.
4. Keep `N = 50` as the first real-guidance baseline before increasing complexity.

## 8. Reentry Burn Planner, Planning-Version v1

### 8.1 Purpose

The reentry burn planner is not a long-horizon trajectory optimizer.
It is a small online optimizer whose job is to choose a burn window and a post-burn target state that make the later landing-burn problem easy and fuel-efficient.

### 8.2 Decision Variables

The reentry burn planner explicitly decides:

1. `t_ign`: ignition time relative to the current state
2. `tau`: burn duration
3. `x_post`: target post-burn state

Recommended post-burn target state:

```text
x_post = [h_post, s_post, vh_post, vs_post, m_post]
```

### 8.3 Why A Hybrid Optimizer

`t_ign` and `tau` are scalar timing decisions and are naturally nonconvex.
`x_post` and feasibility margins are better handled with a convex core.

To stay on a `CVXPYGEN` path, use a hybrid architecture:

1. Outer search over a small bounded set or trust region for `t_ign` and `tau`
2. Inner convex solve for `x_post` and feasibility slack variables

This gives a small online optimizer while staying compatible with `CVXPYGEN`.

### 8.4 Recommended Reentry Optimizer Structure

At each planning cycle:

1. Start from current state `x_now`
2. Generate a reference ballistic trajectory without reentry burn
3. Define a narrow ignition-time window and duration window around the current reference or previous solution
4. For each candidate `(t_ign, tau)` in that small window:
   1. Propagate from `x_now` to ignition under coast dynamics
   2. Build a convex reachable envelope for `x_post`
   3. Solve a small `CVXPYGEN` QP for `x_post`
   4. Evaluate cost
   5. Keep top candidates
5. For the top candidates, run a full landing-burn feasibility check using the `GFOLD P4 N=50` planner
6. Pick the minimum-cost feasible candidate

### 8.5 Reentry Optimizer Inner Problem

For a fixed `(t_ign, tau)` candidate, the inner convex problem chooses `x_post`.

Decision vector:

```text
y = [x_post, slack_q, slack_heat, slack_land, slack_mass]
```

Parameters passed to the QP:

1. Current state `x_now`
2. Coast-to-ignition propagated state `x_ign`
3. Burn duration `tau`
4. Reachable-state linearization or reachable-state polytope
5. Heat and dynamic-pressure surrogate coefficients
6. Landing-burn feasibility surrogate coefficients
7. Fuel model coefficients

### 8.6 Inner Constraints

The inner QP constrains:

1. `x_post` must lie inside the reachable set for the selected `(t_ign, tau)`
2. `m_post` must be consistent with fuel used during the burn
3. `h_post > 0`
4. Vertical and horizontal velocities must remain inside a valid reentry handoff envelope
5. Any heat / dynamic-pressure limit can be softened with slacks instead of becoming immediate infeasibility

Recommended reachability model:

1. Use a linearized finite-burn map around the previous accepted solution
2. Or use a convex reachable polytope precomputed from thrust bounds over duration `tau`

Both options are `CVXPYGEN`-friendly.

### 8.7 Reentry Cost Function

The reentry burn objective must simultaneously consider:

1. Heat / dynamic pressure
2. Landing-point reachability
3. Propellant margin
4. Landing-burn feasibility

Recommended total cost:

```text
J = w_q * J_q
  + w_h * J_heat
  + w_r * J_reach
  + w_p * J_prop
  + w_l * J_land
```

#### 8.7.1 Heat / Dynamic Pressure Term

Use a convex or piecewise-convex surrogate:

```text
J_q = softplus(q_peak - q_ref)
J_heat = softplus(H_peak - H_ref)
```

or

```text
J_aero = w_qint * sum(q_k dt) + w_hint * sum(H_k dt)
```

Recommended planning-period surrogate:

1. Penalize peak `q`
2. Penalize peak heat surrogate `H = rho^0.5 * v^3`
3. Add optional integrated aero load penalty

#### 8.7.2 Landing Reachability Term

This term measures whether the post-burn state still leaves a reachable landing corridor.
It does not mean reachability to a prescribed touchdown point.

Recommended form:

```text
J_reach = penalty_if(h_post, vh_post, vs_post, m_post) leaves handoff envelope
```

Use:

1. A convex distance-to-envelope penalty
2. Or a prefit affine / quadratic surrogate learned from sampled feasible handoff states

#### 8.7.3 Propellant Margin Term

This term pushes the reentry burn to avoid wasting fuel:

```text
J_prop = fuel_used_entry + lambda_margin * max(0, m_required_min - m_post)
```

The design intent:

1. Do not optimize reentry burn only for thermal relief
2. Preserve enough fuel for landing
3. Avoid carrying unnecessary excess into landing if it hurts the rest of the profile

#### 8.7.4 Landing Burn Feasibility Term

This is the most important downstream-coupling term.

Planning-version rule:

1. Run a cheap landing-feasibility surrogate for every reentry candidate
2. Run actual `GFOLD P4 N=50` verification for the top candidates

Recommended cost:

```text
 J_land = surrogate_distance_to_GFOLD_feasible_set
```

And for final verification:

1. If `GFOLD` is infeasible, reject candidate
2. If feasible, include terminal-mass margin in tie-breaking

### 8.8 Reentry Planner Output

The reentry planner returns:

1. `t_ign`
2. `tau`
3. `x_post`
4. Predicted ignition state
5. Predicted post-burn state
6. Predicted heat / dynamic-pressure metrics
7. Predicted landing-burn handoff state or handoff corridor
8. Candidate score breakdown

## 9. Interface Between Reentry Burn And Landing Burn

The reentry planner must not output only a burn time.
It must output a landing-planner-compatible handoff target.

Recommended interface object:

```text
ReentryBurnPlan {
    bool feasible;
    double t_ign;
    double tau;
    State2D ignition_state;
    State2D post_burn_state;
    double q_peak;
    double heat_peak;
    double fuel_used;
    double score_total;
    double score_q;
    double score_heat;
    double score_reach;
    double score_prop;
    double score_land;
}
```

Landing planner interface object:

```text
LandingBurnPlan {
    bool feasible;
    double t_handoff;
    double tf;
    double terminal_mass;
    State2D handoff_state;
    GFOLDConfig cfg;
    GFOLDSolution solution;
}
```

## 10. Current Planner Integration Plan

For the current planner, implementation order should be:

1. Add a 2D recovery propagation module shared by both burns.
2. Add landing-burn planning first, using the `GFOLD P4 N=50` 2D free-downrange variant.
3. Add the reentry burn online optimizer as an outer timing search plus inner `CVXPYGEN` QP.
4. Use the landing planner as the final feasibility oracle for reentry-burn candidate selection.

Current planner runtime flow:

1. Separation state from ascent
2. Propagate ballistic descent
3. Reentry optimizer picks `(t_ign, tau, x_post)`
4. Apply burn in propagation
5. Propagate to landing-burn handoff
6. Solve `GFOLD P4 N=50`
7. Plot full recovery trajectory

## 11. Future Real-Guidance Integration Plan

The real-guidance version should reuse the same module boundaries:

1. Replace simulated state with filtered navigation state
2. Replan reentry burn at low rate, such as 1 to 5 Hz
3. Replan landing burn at low rate, such as 2 to 10 Hz
4. Track the returned powered-phase plan with the flight controller

Important design rule:

1. The real-guidance version should not introduce a different optimizer contract.
2. It should only change the state source, model fidelity, and replanning cadence.

## 12. CVXPYGEN Continuity Rule

Future planning should continue to use `CVXPYGEN`.

The intended split is:

1. Landing burn:
   continue through the `GFOLD + CVXPYGEN` generated backend path
2. Reentry burn:
   use a small `CVXPYGEN` QP core with an outer bounded search on timing variables

This keeps:

1. Fast generated solvers
2. Repeatable problem structure
3. A clear migration path from planner to real guidance

## 13. Concrete Implementation Tasks

### 13.1 Landing Burn

1. Add a new solver generation mode in `GFOLD/solver_py/solver.py` for:
   `p4_n50_2d_free_x`
2. Modify terminal constraints:
   keep `h_N = 0`, `vh_N = 0`, `vs_N = 0`
3. Remove hard downrange terminal equality
4. Generate code with `CVXPYGEN`
5. Add new CMake variant under `GFOLD/cpg_solver/variants/`
6. Add a planner-facing wrapper that maps `[h, s, vh, vs, m]` into `GFOLDConfig`

### 13.2 Reentry Burn

1. Add a small online optimizer module with:
   outer search on `(t_ign, tau)`
2. Add an inner `CVXPYGEN` QP for `x_post`
3. Add aero load surrogate evaluation
4. Add landing-feasibility surrogate evaluation
5. Add top-candidate `GFOLD` verification

### 13.3 Planner Integration

1. Add recovery-specific 2D propagation utilities
2. Add report fields for:
   `t_ign`, `tau`, `x_post`, heat, q, landing feasibility
3. Add plot series for:
   ballistic, reentry burn, coast, landing burn

## 14. Acceptance Criteria

This design is considered successfully implemented when:

1. The planner can compute a reentry-burn plan and a landing-burn plan in one chain.
2. The landing burn uses `GFOLD P4 N=50` in 2D.
3. The landing burn does not constrain touchdown downrange position.
4. The landing burn does constrain terminal altitude to zero.
5. The reentry burn optimizer explicitly chooses `t_ign`, `tau`, and a post-burn target state.
6. The reentry cost includes:
   heat / dynamic pressure, landing reachability, propellant margin, and landing-burn feasibility.
7. The implementation remains on a `CVXPYGEN` path for future solver evolution.

## 15. Defaults Chosen In This Plan

To keep the spec decision-complete, the following defaults are chosen:

1. State dimension for planning: 2D
2. Landing solver family: `GFOLD P4`
3. Landing node count: `N = 50`
4. Landing terminal downrange: free
5. Landing terminal altitude: constrained to zero
6. Landing terminal velocities: constrained to zero
7. Reentry optimizer structure: outer bounded search plus inner `CVXPYGEN` QP
8. Top reentry candidates: verified with actual landing-burn `GFOLD`
9. Future path: continue using `CVXPYGEN` for both the landing solver and the reentry convex core
