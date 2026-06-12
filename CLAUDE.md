# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

FTC (FIRST Tech Challenge) robot code for the DECODE (2025-2026) season. Built on the public FTC SDK (`org.firstinspires.ftc:*:11.0.0`). Produces an Android app that runs on the Robot Controller phone/Control Hub. **All team-authored code lives in the `:TeamCode` module** under `org.firstinspires.ftc.teamcode`. The `:FtcRobotController` module and `build.common.gradle` are SDK-provided — avoid editing them.

## Build & deploy

```bash
./gradlew assembleDebug        # build the TeamCode APK
./gradlew :TeamCode:compileDebugJavaWithJavac   # fast compile check of team code only
./gradlew installDebug         # build + install onto a connected Control Hub / RC phone (adb)
```

There is no unit test suite — OpModes are validated by running them on the robot via the Driver Station. Day-to-day work is normally done in Android Studio (Ladybug 2024.2+); deploy with the Run button or `adb` over Wi-Fi/USB to the Control Hub.

## Key dependencies (beyond the FTC SDK)

- **SolversLib** (`org.solverslib:core`, a FTCLib fork) — the command-based framework. `Command`, `SubsystemBase`, `CommandScheduler`, `GamepadEx`. Package: `com.seattlesolvers.solverslib`.
- **Pedro Pathing** (`com.pedropathing:ftc:2.x`) — the follower / path-following + odometry stack. `Follower`, `Pose`, `BezierLine`, `pathBuilder()`.
- **Lazar Panels** (`com.bylazar:*`) — live dashboard for telemetry and runtime-tunable fields. Classes annotated `@Configurable` expose their `public static` fields for live editing; `JoinedTelemetry` writes to both Panels and the Driver Station. Maven repos: `mymaven.bylazar.com`, `repo.dairy.foundation`.

## Architecture

The codebase deliberately avoids putting logic in OpModes. Instead there is one central robot class and a thin OpMode base that drives its lifecycle.

### Lifecycle: `MyCommandOpMode` → `MyRobot`

`config/core/MyCommandOpMode` extends SolversLib's `CommandOpMode` and defines the loop skeleton. Every OpMode extends it and only needs to construct `r = new MyRobot(...)` in `initialize()` and wire gamepad calls in `run()`. The base calls, in order:
- init loop: `r.startInitLoop()` → `initialize_loop()` → `r.endInitLoop()`
- on start: `atStart()` → `r.onStart()`
- main loop: `r.startPeriodic()` → `run()` → `r.endPeriodic()`
- always: `r.stop()` → `end()` → scheduler `reset()`

`startPeriodic`/`endPeriodic` run the `CommandScheduler`, clear Lynx bulk-read caches, update the follower, run shoot-on-the-move planning, and push telemetry. `run()` in a concrete OpMode only adds gamepad-driven behavior on top.

### `MyRobot` is the central hub

`config/core/MyRobot` extends SolversLib `Robot` and owns every subsystem (`intake`, `turret`, `shooter`, `door`, `storage`, `lift`, `ll`), the Pedro `Follower`, gamepads (`GamepadEx g1/g2`), and the `ShotPlanner`. It holds field geometry constants, alliance state (`isRed`), and cross-OpMode static state carried between auto and teleop: `autoEndPose`, `currentMotif`, `endTurretWrapCount`.

### Subsystem toggling: `SubsystemConfig`

Subsystems are not always all present (test OpModes, partial robots). `MyRobot`'s constructor takes a `List<SubsystemConfig>` (enum: `INTAKE, TURRET, SHOOTER, SPINDEX, LIFT, LL, DOOR, FOLLOWER`); only listed subsystems are instantiated. Guard all subsystem access with `hasSubsystem(...)` / `hasSubsystems(...)`. When adding logic that touches a subsystem, gate it the same way or partial-robot OpModes will NPE.

### Subsystems: `SubsysCore`

Subsystems in `config/subsystems/` extend `config/core/SubsysCore` (a `SubsystemBase`). `SubsysCore` exposes the shared `HardwareMap h` and `JoinedTelemetry t` as statics, set once via `SubsysCore.setGlobalParameters(...)` in the `MyRobot` constructor — so subsystems read hardware/telemetry without taking them as constructor args.

### Commands

`config/commands/` holds `Command` classes (e.g. `OuttakeCommandTele`, `OuttakeCommandAuto`, `TransferCommand`, `FollowPathCommand`) that compose subsystem actions. The shooting flow has separate teleop vs. auto variants (`OuttakeCommandTele` / `OuttakeCommandAuto`). Schedule via `r.schedule(...)` or `CommandScheduler.getInstance()`.

### Shoot-on-the-move (SOTM) & localization

`config/core/util/ShotPlanner` computes a "virtual goal" (lead) from robot pose + velocity so the turret/shooter can fire while moving; `MyRobot.startPeriodic` feeds the result into `turret.input(...)` and `shooter.setPlannedShot(...)`. Localization fuses Pinpoint odometry with Limelight vision — see `config/hardware/PinpointVisionLocalizer`, `pedroPathing/MyFusionLocalizer`, and the EKF/fusion utilities in `config/core/util/` (`PoseEKF`, `PoseFusionFilter`, `VisionMeasurement`). `pedroPathing/Constants` builds the configured `Follower` and holds `fusionLocalizer`.

### OpModes

`opmodes/teleop/` and `opmodes/auto/` are the registered entry points (`@TeleOp` / `@Autonomous`). `MainTeleOp` is the competition teleop; the many `*Auto` classes are different autonomous routines/starting positions. `pedroPathing/Tuning` and the `*Test` OpModes are for tuning/diagnostics. Pre-built path geometry lives in `config/paths/AutoPaths*`.

## Conventions

- Tunable constants are `public static` fields on `@Configurable` classes so they can be edited live in Panels without recompiling — keep this pattern for anything you'd want to tune on the field.
- Field/robot dimensions and offsets are in inches; headings in radians (`Math.toRadians(...)` at call sites).
- New telemetry goes through `r.t` (`JoinedTelemetry`), not raw `telemetry`, so it shows in both Panels and the Driver Station.
