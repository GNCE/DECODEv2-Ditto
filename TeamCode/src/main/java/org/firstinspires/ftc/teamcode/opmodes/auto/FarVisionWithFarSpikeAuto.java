package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Alliance;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

import java.util.List;

@Autonomous(group="Far Auto", name="Far Vision + Far Spike Auto")
public class FarVisionWithFarSpikeAuto extends MyCommandOpMode {
    AutoPaths autoPaths;

    // Hard safety ceiling on the robot's field y. If y ever exceeds this, the routine aborts and
    // the drivetrain brakes. NOTE: this triggers AFTER y crosses the line, so the robot will
    // overshoot slightly while braking -- set this a touch below the true hard limit if you need
    // to guarantee y never physically passes 36.
    public static double MAX_Y = 36;

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER, SubsystemConfig.LL), OpModeType.AUTO);
    }

    @Override
    public void atStart() {
        autoPaths = new AutoPaths(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
        r.overrideAutoEndPose(autoPaths.getPose(AutoPaths.PoseId.START_BACK));
        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                                new InstantCommand(() -> r.shooter.setActive(true)),
                                new InstantCommand(() -> r.ll.setMode(Limelight.Mode.BALL_DETECTION))
                        ),
                        new WaitCommand(250),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        // Keep the flywheel spun for the next shot pose while driving in to collect.
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_1)),
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.START_BACK_TO_HUMAN_PLAYER_END)),
                                        new WaitCommand(500)
                                ),
                                new WaitUntilCommand(() -> r.storage.getSize() == 3)
                        ),
                        r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_1)),
                        new WaitCommand(100),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_3)),
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_BACK_1_TO_FAR_SPIKE_END)),
                                        new WaitCommand(300)
                                ),
                                new WaitUntilCommand(() -> r.storage.getSize() == 3)
                        ),
                        r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_3)),
                        // Scan for balls during this shot so the collect that follows needs no scan wait.
                        new ParallelCommandGroup(
                                r.shootAll(),
                                r.scanForBalls()
                        ),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        // Every loop shoots + scans from SHOOT_BACK_SCAN, so hold the flywheel there throughout.
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_SCAN)),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        new ParallelRaceGroup(
                                                r.collectThreeBallsNoScan(),
                                                new WaitUntilCommand(() -> r.storage.getSize() == 3)
                                        ),
                                        r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_SCAN)),
                                        // Shoot and re-scan at the same time, so the next loop's collect is ready immediately.
                                        new ParallelCommandGroup(
                                                r.shootAll(),
                                                r.scanForBalls()
                                        ),
                                        new InstantCommand(() -> r.door.setOpen(false)),
                                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE))
                                ),
                                6
                        )
                )
                        .raceWith(new WaitCommand(29500))
                        // Y-LIMIT SAFETY: abort the whole routine the instant the robot's y exceeds MAX_Y.
                        .raceWith(new WaitUntilCommand(() -> r.f.getPose().getY() > MAX_Y))
                        .andThen(r.clearShooterSpinUp())
                        // If we exited because of the y-limit, brake + hold instead of driving to park
                        // (so we don't drive back across the line). Otherwise park as normal.
                        .andThen(new ConditionalCommand(
                                new InstantCommand(() -> {
                                    r.f.startTeleopDrive();
                                    r.f.setTeleOpDrive(0, 0, 0, true);
                                }),
                                r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.PARK_FINAL)),
                                () -> r.f.getPose().getY() > MAX_Y
                        ))
        );
    }
}
