# Falcon 9-Like Liftoff Algorithm

## Goal

This document describes the ascent algorithm now intended for the planner.

It does **not** aim to be a mathematically pure UPFG implementation.
Instead, it aims to imitate the structure of a Falcon 9-style ascent program while still keeping a terminal-state correction layer so the planner can:

1. maximize insertion mass,
2. keep stage-1 recovery feasible,
3. maintain acceptable stage-separation conditions for stage 2.

The algorithm is designed for the current 2D planner, but the structure is also suitable as a later bridge toward a higher-fidelity guidance implementation.

## Design Principles

The ascent law follows four rules:

1. Use a Falcon 9-like liftoff and gravity-turn backbone rather than pure end-state chasing from T+0.
2. Keep Max-Q throttle-bucket behavior explicit.
3. Add terminal correction late enough that it shapes separation conditions without destroying the early ascent profile.
4. In max-payload mode, search should prefer solutions that reduce unnecessary stage-1 reserve while still keeping stage-2 insertion acceptable.
5. Keep stage-separation flight-path angle low enough that stage 2 does not waste dV climbing above target orbit and then falling back.

## Phase Structure

The ascent is split into these phases:

1. Vertical hold / initial rise
2. Pitch-kick and gravity-turn build-up
3. Max-Q throttle bucket
4. Mid-ascent pitch program with growing terminal correction
5. Late-ascent terminal shaping to target stage separation
6. MECO
7. Coast to separation

## State And Controls

The planner remains in 2D:

```text
state = [x, z, vx, vz, m]
```

Where:

1. `x`: downrange distance
2. `z`: altitude
3. `vx`: horizontal velocity
4. `vz`: vertical velocity
5. `m`: total vehicle mass

Control variables:

1. throttle command
2. pitch command

## Core Guidance Logic

### 1. Liftoff Backbone

The ascent should begin with a Falcon 9-like liftoff backbone:

1. Hold near-vertical attitude immediately after liftoff
2. Introduce a gentle pitch kick after the first few seconds
3. Continue with a smooth gravity turn rather than an aggressive immediate terminal pointing law

This backbone exists because Falcon 9 ascent is not flown as "pure terminal guidance from the pad."
The real vehicle uses an ascent program shape that builds aerodynamic and gravity-turn behavior first, then refines terminal conditions later.

### 2. Pitch Program

The pitch backbone should be generated from a blend of:

1. time-based program,
2. altitude-based program.

Recommended use:

```text
pitch_prog = w_t * pitch_time + w_h * pitch_alt
```

Where:

1. `pitch_time` controls early rise and gravity-turn pacing
2. `pitch_alt` prevents obviously unrealistic pitch behavior versus altitude

This is the "Falcon 9-like" part of the ascent law.

### 3. Terminal Correction Layer

Terminal correction should not dominate the entire ascent.
It should ramp in progressively after the vehicle is already committed to the ascent plane and gravity turn.

Recommended structure:

```text
pitch_cmd = blend(pitch_prog, pitch_terminal_feedback, w_fb)
```

Where:

1. `w_fb` is small early,
2. grows through mid-ascent,
3. becomes dominant only late enough to shape separation conditions.

This gives the planner a "program first, correction second" behavior.

### 4. Max-Q Throttle Bucket

Max-Q handling remains explicit.

The throttle law should:

1. identify a target dynamic-pressure band,
2. reduce throttle inside the bucket window,
3. return toward full throttle after the bucket.

Recommended behavior:

1. full throttle at liftoff,
2. throttle reduction only around the bucket region,
3. near-full throttle again afterward unless terminal shaping requires a small reduction.

This is important because a Falcon 9-like ascent should visibly behave like a launch vehicle with a real Max-Q bucket, not like an optimizer continuously modulating thrust everywhere.

## Terminal Targets

The ascent still needs terminal targets for stage separation.

Use soft references for:

1. separation altitude,
2. separation speed,
3. horizontal velocity ratio,
4. separation flight-path angle,
4. approximate Max-Q timing,
5. approximate MECO / separation timing.

Important rule:

1. these are **soft references**,
2. not hard mission constraints for normal mission planning.

That means:

1. normal planning should optimize mission success first,
2. Falcon-like timing and profile resemblance should be advisory, not mandatory.

## Search Logic

The stage-1 ascent search should be mission-driven.

### Normal Mission Mode

Objective order:

1. acceptable stage-2 insertion
2. acceptable stage-1 recovery
3. reasonable separation state
4. lower profile penalty only as a soft preference

This means the planner should never reject a good mission solution only because it is not close enough to one particular Falcon 9 mission window.

### Max Payload Mode

Objective order:

1. acceptable stage-2 insertion
2. acceptable stage-1 recovery
3. minimize **excess** stage-1 reserve beyond what recovery actually needs

This is critical.

The max-payload solver must not behave like a conservative mission planner.
Once recovery remains feasible, it should stop rewarding large leftover stage-1 propellant and instead push that margin back into insertion performance.

## Stage-2 Coupling

The stage-1 ascent algorithm must be tuned together with stage-2 insertion.

Reason:

1. separation speed too low starves stage 2,
2. separation altitude too low hurts stage 2,
3. separation flight-path angle too high forces stage 2 to spend dV on vertical cleanup,
4. excessive vertical energy can push stage 2 above target orbital altitude before it settles back,
5. separation speed too high can hurt stage-1 recovery,
6. stage-1 burn search must therefore evaluate real stage-2 propagation, not ideal dV only.

So the ascent search is coupled to the propagated stage-2 solver:

1. candidate stage-1 ascent
2. real stage-2 insertion propagation
3. insertion error evaluation
4. recovery reserve evaluation
5. final candidate ranking

## Current Repo Behavior

The current implementation in this repository follows this practical sequence:

1. `MissionRequest` is sanitized first. UTC launch time is converted to Earth rotation / launch-window geometry before the ascent search is evaluated.
2. Stage 1 starts from site latitude and includes the rotational velocity component available for the requested inclination.
3. Early ascent uses a time/altitude pitch backbone with an explicit Max-Q throttle bucket.
4. Normal recoverable missions cap usable stage-1 propellant by the configured stage-1 reserve ratio. The cap is bypassed only for explicit burnout / no-recovery modes.
5. Late stage-1 ascent uses a terminal correction layer toward separation altitude, speed, and flight-path angle. This layer is deliberately not a full stage-1 UPFG handoff because preserving recovery margin and avoiding overly lofted separation are more important in the current 2D planner.
6. Every candidate stage-1 separation is propagated through the real stage-2 solver before ranking. A separation that looks good by altitude/speed alone is not accepted if stage 2 cannot insert accurately.
7. Stage 2 uses blended terminal UPFG guidance, adaptive terminal step size, and a continuous orbit-error penalty. The stage-1 search therefore sees actual insertion miss distance instead of an idealized delta-v estimate.
8. Separation-time samples can be evaluated in parallel. Max-payload mode also uses a parallel coarse payload sweep before the final fine search.
9. Candidate ranking prefers feasible orbit insertion and recovery first, then uses remaining stage-2 propellant, stage-1 reserve, Max-Q excess, and separation-state error as secondary terms.

In short, the liftoff algorithm is now a coupled mission planner:

```text
UTC / target orbit
  -> stage-1 pitch + throttle candidate
  -> separation state
  -> propagated stage-2 UPFG insertion
  -> propagated stage-1 recovery
  -> candidate ranking
```

This is intentionally more conservative than a pure single-stage guidance law, but more useful for payload and recovery trade studies.

## What This Algorithm Is Not

This ascent law is not:

1. a strict textbook UPFG implementation,
2. a hard-coded copy of one Falcon 9 mission timeline,
3. a pure heuristic pitch table with no terminal feedback.

It is instead:

1. Falcon 9-like liftoff and gravity-turn programming,
2. explicit Max-Q throttling,
3. late terminal correction,
4. mission-driven selection with payload-aware search modes.

## Implementation Notes For This Repo

In the current planner, the intended implementation behavior is:

1. early ascent follows a Falcon-like pitch backbone,
2. Max-Q bucket is explicit,
3. terminal correction shapes separation state late,
4. stage-separation flight-path angle is actively driven downward to avoid an overly lofted stage-2 injection,
5. stage-2 insertion is propagated and fed back into stage-1 burn search,
6. stage-2 UPFG is used as a terminal insertion correction rather than a replacement for the whole ascent profile,
7. max-payload mode prioritizes reducing unnecessary stage-1 reserve,
8. launch-window calculations are UTC-driven when a UTC epoch is provided in the vehicle configuration.

## Acceptance Criteria

The liftoff algorithm should be considered correct for this planner when:

1. the ascent visually resembles a launch-vehicle gravity turn,
2. Max-Q occurs through a recognizable throttle-bucket mechanism,
3. stage separation remains in a useful altitude/speed/gamma band,
4. stage 2 does not significantly overshoot orbital altitude before converging,
5. stage 2 can reach target orbit with propagated guidance,
6. max-payload mode no longer preserves large unnecessary stage-1 reserve,
7. normal mission mode does not force one exact Falcon 9 mission window.
