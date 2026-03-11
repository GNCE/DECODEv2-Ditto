package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
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
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

import java.util.List;

@Autonomous(group="Far Auto", name="Far With Far Spike Auto")
public class FarWithFarSpikeAuto extends MyCommandOpMode {
    AutoPaths autoPaths;

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER), OpModeType.AUTO);

        autoPaths = new AutoPaths(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
        r.overrideAutoEndPose(autoPaths.getPose(AutoPaths.PoseId.START_FRONT));
        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                                new InstantCommand(() -> r.shooter.setActive(true))
                        ),
                        new ParallelDeadlineGroup(
                                new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.START_FRONT_TO_MID_SPIKE_END)),
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        r.shootAll(),
                                        new InstantCommand(() -> r.door.setOpen(false))
                                )
                        ),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                                new InstantCommand(() -> r.door.setOpen(false))
                        ),
                        new ParallelDeadlineGroup(
                                new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.MID_SPIKE_END_TO_SHOOT_FRONT)),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> r.f.getPathCompletion() > 0.9),
                                        r.shootAll(),
                                        new InstantCommand(() -> r.door.setOpen(false))
                                )
                        ),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                                                new InstantCommand(() -> r.door.setOpen(false))
                                        ),
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_FRONT_TO_GATE_INTAKE)),
                                        new WaitCommand(2000),
                                        new ParallelCommandGroup(
                                                new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.GATE_INTAKE_TO_SHOOT_FRONT)),
                                                new SequentialCommandGroup(
                                                        new WaitUntilCommand(() -> r.f.getPathCompletion() > 0.9),
                                                        r.shootAll()
                                                )
                                        )
                                ), 3
                        )
                )
        );
    }
}
