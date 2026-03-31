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
import org.firstinspires.ftc.teamcode.config.paths.AutoPaths2;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

import java.util.List;

@Autonomous(group="Close Auto", name="Donut")
public class I_LOVE_DONUTS extends MyCommandOpMode {
    AutoPaths2 autoPaths2;

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER), OpModeType.AUTO);
    }

    @Override
    public void atStart() {
        autoPaths2 = new AutoPaths2(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
        r.overrideAutoEndPose(autoPaths2.getPose(AutoPaths2.PoseId.SINGLE_FRONT_START));
        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                                new InstantCommand(() -> r.shooter.setActive(true))
                        ),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_CLOSE_START_TO_MID_SPIKE_START)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),

                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_MID_SPIKE_START_TO_MID_SPIKE_END)),
                        new WaitCommand(75),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_MID_SPIKE_END_TO_FRONT_SHOOT_AFTER_GATE)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),

                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SHOOT_TO_GATE_INTAKE_NORMAL)),
                        new WaitCommand(250),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.TRIPLE_GATE_INTAKE_TO_GATE_INTAKE_SAFE)),
                        new WaitCommand(650),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.TRIPLE_GATE_SAFE_TO_GATE_INTAKE_SAFE_SAFE)),
                        new WaitCommand(125),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.GATE_INTAKE_SAFE_SAFE_TO_SHOOT)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),

                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SHOOT_TO_GATE_INTAKE_NORMAL)),
                        new WaitCommand(675),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.TRIPLE_GATE_INTAKE_TO_GATE_INTAKE_SAFE)),
                        new WaitCommand(700),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.TRIPLE_GATE_SAFE_TO_GATE_INTAKE_SAFE_SAFE)),
                        new WaitCommand(125),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.GATE_INTAKE_SAFE_SAFE_TO_SHOOT_FINAl)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),

                        /*
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SHOOT_TO_GATE_INTAKE_NORMAL)),
                                        new WaitCommand(450),
                                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.TRIPLE_GATE_INTAKE_TO_GATE_INTAKE_SAFE)),
                                        new WaitCommand(1000),
                                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.TRIPLE_GATE_INTAKE_SAFE_TO_SHOOT)),
                                        r.shootAll2(),
                                        new InstantCommand(() -> r.door.setOpen(false))
                                ),
                                2
                        ),

                         */

                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_SHOOT_AFTER_GATE_TO_FAR_SPIKE_END)),
                        new WaitCommand(70),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_FAR_SPIKE_END_TO_FRONT_SHOOT_AFTER_GATE)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),


                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_SHOOT_AFTER_GATE_TO_CLOSE_SPIKE_END)),
                        new WaitCommand(25),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_CLOSE_SPIKE_END_TO_CLOSE_FINAL_SHOOT)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false))

                )
        );
    }
}
