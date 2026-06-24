---
name: ditto-sotm-improvements
description: SOTM aim/lead improvements made to ShotPlanner (latency model + decel-to-stop)
metadata:
  type: project
---

SOTM work on `ShotPlanner.java` (2026-06), focused on AIM/LEAD math only (firing gates are intentional — see [[shooter-sotm-design-intent]]):

- **Explicit latency model** replacing the single `SHOT_PREDICTION_SEC`. Two INDEPENDENT (not additive) horizons, each actuator led by its own delay:
  - `RELEASE_LATENCY_SEC` (~0.03, small): "run transfer" call → ball actually leaving. The ball sits against the flywheel so this is tiny. Drives shot geometry + the lead's launch velocity (ball inherits near-current velocity).
  - `TURRET_LATENCY_SEC` (~0.15, the dominant one): the servo response delay. Turret aim is led by exactly this so the slow servo tracks the target in real time (command now = angle the servo reaches τ later). Release latency is much LESS than turret latency (per user).
  - `ShotCommand` carries `predictedTurretPose` (turret horizon) separate from `predictedRobotPose` (release horizon).
- **Stop-clamped motion prediction** (`integrateAxisStopClamped`/`predictWithStop`): per-axis integration that coasts a braking robot to a stop and holds it instead of extrapolating through zero into a fake reversal. This is the decel-to-stop-then-accelerate fix.
- **CRITICAL CONTEXT: loop time is ~40ms (25Hz).** A velocity sample arrives only every 40ms and is already filtered, so odometry-derived ACCELERATION (difference of two such samples) is ~40-80ms laggy and noisy — it cannot track a quick decel/reverse. No regression window is good (2 samples = noise, 4-5 samples = 160-200ms lag).
- **The decel over-lead was caused by feeding that bad accel into the forward prediction**: during accel→brake the laggy accel still reports the old (speeding-up) value, so the turret/lead extrapolate a FASTER velocity and aim off to the side. User symptom: "decelerate, click launch, turret compensates for a faster velocity and shoots off to the side."
- **Accel IS needed** for the turret: over the ~0.15s servo delay (≈4 loops) the target moves enough that ignoring accel mis-leads the turret on a decel. The fix was the SOURCE, not removing accel.
- **`updateAcceleration`: fresh ONE-STEP finite diff (this loop's velocity − last) + light EMA** (`ACCEL_SMOOTHING_ALPHA`, ~0.4), replacing the old 6-sample/0.2s regression. One-step lags ~1 loop instead of ~5, so it tracks the brake; EMA + the slow servo tame noise. `USE_ACCEL_PREDICTION` (default TRUE) gates it; false = constant-velocity fallback. The old `MotionSample`/regression/`ACCEL_REGRESSION_WINDOW_SAMPLES`/`MAX_REGRESSION_HISTORY_SEC` are gone.
- **Velocity-lag compensation now applied to the TURRET pose, not just the lead** (fix for hard-braking misses). Pedro's velocity reads TOO FAST during a brake ("robot thinks it's going faster than it is"); `compensateVel(v, accel, VELOCITY_LATENCY_SEC)` shifts it to the true current value (won't flip sign past a stop) and is fed into ALL predictions (turret pose, shot pose, lead). Earlier the turret pose used raw laggy velocity projected over the 0.15s servo horizon → amplified over-lead. Raised `ACCEL_SMOOTHING_ALPHA` 0.4→0.6 so accel/comp react faster to the brake.
- Tuning braking misses: still over-leads → raise `VELOCITY_LATENCY_SEC` and/or `ACCEL_SMOOTHING_ALPHA` (toward 1.0), and/or lower `TURRET_LATENCY_SEC`; under-leads → lower `VELOCITY_LATENCY_SEC`. Run telemetry OFF for the fastest loop. "SOTM Launch Vel" should read LOWER than "SOTM Raw Vel" during a brake.
- `VELOCITY_LATENCY_SEC` / `RELEASE_LATENCY_SEC` only matter when accel is ON (with accel off, the comp + projection are no-ops, so Launch Vel == Raw Vel — expected).
- Debug telemetry in MyRobot: "SOTM Launch Vel" vs "SOTM Raw Vel", "SOTM Est Accel" vs "SOTM Pedro Accel" (Pedro's `f.getAcceleration()` is a fresh 1-step diff), "SOTM Lead X/Y".
- Hood/flight-time/lead still solved at CURRENT measured RPM (kept intentional design).

Gated by `ENABLE_FUTURE_POSE_PREDICTION` (FPP). In auto these are only enabled if the OpMode calls `ShotPlanner.enableSOTM()/enableFPP()` (teleop toggles them on the g1 right trigger). `SOTM_Test` now enables them.

Tune on field: `TURRET_LATENCY_SEC` (measure servo slew+transport delay), `RELEASE_LATENCY_SEC`.
