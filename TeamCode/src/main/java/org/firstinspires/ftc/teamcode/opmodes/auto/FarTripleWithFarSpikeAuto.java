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

@Autonomous(group="Far Auto", name="Far Triple Far Spike Auto")
public class FarTripleWithFarSpikeAuto extends MyCommandOpMode {
    AutoPaths autoPaths;

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER), OpModeType.AUTO);

        autoPaths = new AutoPaths(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
        r.overrideAutoEndPose(autoPaths.getPose(AutoPaths.PoseId.START_BACK));
        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                                new InstantCommand(() -> r.shooter.setActive(true))
                        ),
                        new WaitCommand(250),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.START_BACK_TO_HUMAN_PLAYER_END)),
                        new WaitCommand(500),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.HP_TO_SHOOT_BACK_1)),
                        new WaitCommand(250),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_BACK_1_TO_FAR_SPIKE_END)),
                        new WaitCommand(500),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.FAR_SPIKE_END_TO_SHOOT_BACK_2)),
                        new WaitCommand(500),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_BACK_2_TO_HP_END)),
                        new WaitCommand(500),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.HP_END_TO_SHOOT_BACK_3)),
                        new WaitCommand(500),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_BACK_3_TO_GATE_SWEEP_END)),
                                        new WaitCommand(500),
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.GATE_SWEEP_END_TO_SHOOT_BACK_3)),
                                        new WaitCommand(500),
                                        r.shootAll(),
                                        new InstantCommand(() -> r.door.setOpen(false)),
                                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE))
                                ),
                                2
                        )
                )
        );
    }
}
