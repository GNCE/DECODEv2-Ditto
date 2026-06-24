---
name: ditto-loop-time-optimization
description: Loop-time optimizations and the telemetry-gating pattern for hardware reads
metadata:
  type: project
---

Loop time was ~40ms (25Hz). Main culprit: motor `getCurrent()` calls — each is a SEPARATE hub round-trip (~2-3ms), NOT part of the bulk read. There were 5/loop, all dead or telemetry-only.

Changes (2026-06):
- Killed dead read: `storage.input(intake.getCurrent(), ...)` → `storage.input(0,0)`. Storage's `curAmps`/`curVelo` were stored but never used.
- Added `SubsysCore.telemetryEnabled` (mirrors `MyRobot.telemetryEnabled`, pushed each loop in `updateTelemetryToggle`). Subsystems gate their telemetry — and especially the telemetry-only hardware reads — on it: Shooter (m1/m2 getCurrent), Intake (im/tr getCurrent), Limelight (getStatus/isConnected/isRunning), plus Turret/Storage/Door/Lift telemetry strings. So match mode (telemetry OFF via g2 OPTIONS / g1 OPTIONS) skips all of it.
- Limelight: `pipelineSwitch` now only fires on change (`switchPipeline`/`currentPipeline`), not every loop; dropped the unused MT1 botpose read (only MT2 feeds the fusion localizer).
- Rumble edge-triggered (`wasRumbling`) instead of re-sending a DS command every loop.
- **Write caching** via new `config/hardware/CachedServo` (mirrors `CachedMotor`): only sends `setPosition` when the value changes. Applied to Door, Shooter hood, Turret s1/s2. Intake pitch and Lift PTO clutch use an inline last-value guard (ServoEx). Big win for servos that hold a constant value (door/pitch/pto); turret+hood still write each loop while actively tracking but are cached when holding. Reads were already bulk-cached (MANUAL mode, one bulk read/loop).

Expected: ~10-15ms off the loop in match mode (telemetry off), plus fewer servo commands.

CAVEAT for SOTM tuning: the SOTM debug telemetry in MyRobot is gated by telemetryEnabled, so tuning happens at ~40ms loops while the match runs faster (telemetry off). Absolute-time latency constants (RELEASE/TURRET_LATENCY_SEC) are loop-rate independent, but the accel EMA's per-loop alpha is not — see [[ditto-sotm-improvements]]. If this bites, exempt the SOTM debug lines from the gate so tuning happens at match-representative loop times.
