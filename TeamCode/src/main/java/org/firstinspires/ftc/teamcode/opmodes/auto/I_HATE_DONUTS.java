package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
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
import org.firstinspires.ftc.teamcode.config.paths.AutoPathsForCloseAuto;
import org.firstinspires.ftc.teamcode.config.paths.AutoPathsForFarSolo;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

import java.util.List;
import java.util.concurrent.locks.Condition;

@Autonomous(group="Far Auto", name="THE Autonomous")
public class I_HATE_DONUTS extends MyCommandOpMode {
    AutoPathsForFarSolo autoPathsForFarSolo;

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER), OpModeType.AUTO);
    }

    @Override
    public void atStart() {
        autoPathsForFarSolo = new AutoPathsForFarSolo(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
        r.overrideAutoEndPose(autoPathsForFarSolo.getPose(AutoPathsForFarSolo.PoseId.START_BACK));
        schedule(
                new SequentialCommandGroup(


                        // SHOOT
                        new ParallelCommandGroup(
                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                                new InstantCommand(() -> r.shooter.setActive(true))
                        ),
                        new WaitCommand(250),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),



                        // GO TO PICKUP 3 AND SHOOT
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(r.f, autoPathsForFarSolo.getPath(AutoPathsForFarSolo.PathId.SHOOT_BACK_1_TO_FAR_SPIKE_END)),
                                        new WaitCommand(300)
                                ),
                                new WaitUntilCommand(() -> r.storage.getSize() == 3)
                        ),
                        r.goToLinear(autoPathsForFarSolo.getPose(AutoPathsForFarSolo.PoseId.SHOOT_BACK_3)),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),



                        // GO TO PICKUP 2 AND SHOOT
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(r.f, autoPathsForFarSolo.getPath(AutoPathsForFarSolo.PathId.SHOOT_BACK_3_TO_MID_SPIKE_END)),
                                        new WaitCommand(300)
                                ),
                                new WaitUntilCommand(() -> r.storage.getSize() == 3)
                        ),
                        r.goToLinear(autoPathsForFarSolo.getPose(AutoPathsForFarSolo.PoseId.SHOOT_CLOSE_1)),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),



                        // GATE CYCLE 1
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(r.f, autoPathsForFarSolo.getPath(AutoPathsForFarSolo.PathId.SHOOT_TO_GATE_INTAKE_NORMAL)),
                                        new WaitCommand(225),
                                        new FollowPathCommand(r.f, autoPathsForFarSolo.getPath(AutoPathsForFarSolo.PathId.TRIPLE_GATE_INTAKE_TO_GATE_INTAKE_SAFE)),
                                        new WaitCommand(725),
                                        new FollowPathCommand(r.f, autoPathsForFarSolo.getPath(AutoPathsForFarSolo.PathId.TRIPLE_GATE_SAFE_TO_GATE_INTAKE_SAFE_SAFE)),
                                        new WaitCommand(100)
                                ),
                                new WaitUntilCommand(() -> r.storage.getSize() == 3)
                        ),
                        r.goToLinear(autoPathsForFarSolo.getPose(AutoPathsForFarSolo.PoseId.FRONT_SHOOT_AFTER_GATE_FINAL_FINAL)),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),



                        // GO TO PICKUP 1 AND SHOOT
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(r.f, autoPathsForFarSolo.getPath(AutoPathsForFarSolo.PathId.SINGLE_SHOOT_AFTER_GATE_TO_CLOSE_SPIKE_END)),
                                        new WaitCommand(300)
                                ),
                                new WaitUntilCommand(() -> r.storage.getSize() == 3)
                        ),
                        r.goToLinear(autoPathsForFarSolo.getPose(AutoPathsForFarSolo.PoseId.SHOOT_CLOSE_1)),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),



                        // GATE CYCLE 2
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(r.f, autoPathsForFarSolo.getPath(AutoPathsForFarSolo.PathId.SHOOT_TO_GATE_INTAKE_NORMAL)),
                                        new WaitCommand(225),
                                        new FollowPathCommand(r.f, autoPathsForFarSolo.getPath(AutoPathsForFarSolo.PathId.TRIPLE_GATE_INTAKE_TO_GATE_INTAKE_SAFE)),
                                        new WaitCommand(725),
                                        new FollowPathCommand(r.f, autoPathsForFarSolo.getPath(AutoPathsForFarSolo.PathId.TRIPLE_GATE_SAFE_TO_GATE_INTAKE_SAFE_SAFE)),
                                        new WaitCommand(100)
                                ),
                                new WaitUntilCommand(() -> r.storage.getSize() == 3)
                        ),
                        r.goToLinear(autoPathsForFarSolo.getPose(AutoPathsForFarSolo.PoseId.SHOOT_BACK_3)),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE))


                        /*

                        // GO TO HP AND SHOOT
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


                        // CURVE FAR PICKUP
                        new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_BACK_3_TO_GATE_SWEEP_END)),
                                        new WaitUntilCommand(() -> r.storage.getSize() == 3)
                                ),
                                r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_2)),
                                new ConditionalCommand(r.shootAll(), new InstantCommand(), () -> r.storage.getSize() >= 2),
                                new InstantCommand(() -> r.door.setOpen(false)),
                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE))
                        ),

                        // STRAIGHT FAR PICKUP
                        new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_BACK_2_ALT_GATE_SWEEP_END)),
                                        new WaitUntilCommand(() -> r.storage.getSize() == 3)
                                ),
                                r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.SHOOT_FINAL)),
                                new ConditionalCommand(r.shootAll(), new InstantCommand(), () -> r.storage.getSize() >= 2),
                                new InstantCommand(() -> r.door.setOpen(false)),
                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE))
                        )
                        */
                )
                        .raceWith(new WaitCommand(29500))
                        .andThen(r.goToLinear(autoPathsForFarSolo.getPose(AutoPathsForFarSolo.PoseId.PARK_FINAL)))
        );
    }
}
