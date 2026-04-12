package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Alliance;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.paths.AutoPathsForCloseAuto;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

import java.util.List;

@Autonomous(group="Close Auto", name="Donut - Partner")
public class I_LOVE_DONUTS_YAY extends MyCommandOpMode {
    AutoPathsForCloseAuto autoPaths2;

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER), OpModeType.AUTO);
    }

    @Override
    public void atStart() {
        autoPaths2 = new AutoPathsForCloseAuto(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
        r.overrideAutoEndPose(autoPaths2.getPose(AutoPathsForCloseAuto.PoseId.SINGLE_FRONT_START));
        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                                new InstantCommand(() -> r.shooter.setActive(true))
                        ),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.SINGLE_CLOSE_START_TO_MID_SPIKE_START)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),


                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.SINGLE_MID_SPIKE_START_TO_MID_SPIKE_END)),
                        new WaitCommand(75),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.SINGLE_MID_SPIKE_END_TO_FRONT_SHOOT_AFTER_GATE)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),


                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.SHOOT_TO_GATE_INTAKE_NORMAL)),
                        new WaitCommand(200),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.TRIPLE_GATE_INTAKE_TO_GATE_INTAKE_SAFE)),
                        new WaitCommand(725),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.TRIPLE_GATE_SAFE_TO_GATE_INTAKE_SAFE_SAFE)),
                        new WaitCommand(75),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.GATE_INTAKE_SAFE_SAFE_TO_SHOOT)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),


                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.SHOOT_TO_GATE_INTAKE_NORMAL)),
                        new WaitCommand(350),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.TRIPLE_GATE_INTAKE_TO_GATE_INTAKE_SAFE)),
                        new WaitCommand(750),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.TRIPLE_GATE_SAFE_TO_GATE_INTAKE_SAFE_SAFE)),
                        new WaitCommand(90),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.GATE_INTAKE_SAFE_SAFE_TO_SHOOT_FINAl)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),


                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.SHOOT_TO_GATE_INTAKE_NORMAL)),
                        new WaitCommand(350),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.TRIPLE_GATE_INTAKE_TO_GATE_INTAKE_SAFE)),
                        new WaitCommand(750),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.TRIPLE_GATE_SAFE_TO_GATE_INTAKE_SAFE_SAFE)),
                        new WaitCommand(90),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.GATE_INTAKE_SAFE_SAFE_TO_SHOOT_FINAl_FINAL)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),


                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.SINGLE_SHOOT_AFTER_GATE_TO_CLOSE_SPIKE_END)),
                        new WaitCommand(25),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPathsForCloseAuto.PathId.SINGLE_CLOSE_SPIKE_END_TO_CLOSE_FINAL_SHOOT)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false))

                )
        );
    }
}
