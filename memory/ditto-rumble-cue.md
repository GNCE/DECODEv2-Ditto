---
name: ditto-rumble-cue
description: Driver rumble — full-magazine buzz + timed shot cue via RumbleManager
metadata:
  type: project
---

`config/core/util/hardware/RumbleManager` centralizes gamepad-1 rumble. Everything is a one-shot effect (no continuous buzz); a hub command is sent only when a pulse starts. Two cues:
- FULL: a single short buzz (`FULL_RUMBLE_MS`, ~300) fired ONCE on the rising edge of `storage.getSize()==3` (was a continuous buzz; user wanted "a little bit, not continuous").
- SHOT: a single short buzz (`SHOT_RUMBLE_MS`, ~250) that PREEMPTS FULL. Since FULL is one-shot it's normally over before a shot; if you fire right after loading, the shot buzz overrides whatever's playing. (Earlier it was a double-tap `runRumbleEffect` to stand out vs the old continuous full buzz; pointless now that both are single buzzes.)

Wiring: `MyRobot.rumbleManager.update(size==3)` each loop. `OuttakeCommandTele` takes an `onLaunch` Runnable fired the instant the transfer pushes the balls (`MyRobot::cueShotRumble` → `triggerShot()`). Auto (`OuttakeCommandAuto`) intentionally has no cue (no driver).

Timing (all @Configurable, tune in Panels): the cue should be FELT as balls clear so the driver can drive off. `triggerShot` is called at transfer-start and schedules the send for `+max(0, BALL_LAUNCH_OFFSET_MS - HUB_RESPONSE_DELAY_MS - EXPECTED_LOOP_MS/2)`. Human reaction time is deliberately NOT subtracted (would cue "go" before balls are out, disturbing the shot); the cue is just long enough to still buzz when they react. Tune `BALL_LAUNCH_OFFSET_MS` (~110), `HUB_RESPONSE_DELAY_MS` (~60), `EXPECTED_LOOP_MS` (~40, loop period). Relates to [[ditto-loop-time-optimization]].
