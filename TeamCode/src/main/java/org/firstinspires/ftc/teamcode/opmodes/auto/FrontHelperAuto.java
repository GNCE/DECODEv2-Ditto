//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
//import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
//import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
//import com.seattlesolvers.solverslib.command.RunCommand;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//import com.seattlesolvers.solverslib.command.WaitUntilCommand;
//
//import org.firstinspires.ftc.teamcode.config.commands.FollowPathCommand;
//import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
//import org.firstinspires.ftc.teamcode.config.core.MyRobot;
//import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
//import org.firstinspires.ftc.teamcode.config.core.util.ArtifactMatch;
//import org.firstinspires.ftc.teamcode.config.core.util.OpModeType;
//import org.firstinspires.ftc.teamcode.config.paths.AutoPaths;
//import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.config.subsystems.Limelight;
//import org.firstinspires.ftc.teamcode.config.subsystems.Turret;
//
//import kotlin.time.Instant;
//
//@Autonomous(group="Partner Auto", name="Front Helper Auto")
//public class FrontHelperAuto extends MyCommandOpMode {
//    AutoPaths autoPaths;
//
//    @Override
//    public void initialize() {
//        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, OpModeType.AUTO);
//    }
//
//    @Override
//    public void initialize_loop() {
//        r.preloadSelection();
//        if(r.spindex.isEmpty()) r.spindex.setIdx(r.spindex.getCloserIndexToEnd());
//        else r.spindex.setIdx(r.spindex.getClosestIndex(ArtifactMatch.NONE));
//        r.spindex.periodic();
//        r.intake.periodic();
//    }
//
//    @Override
//    public void atStart() {
//        autoPaths = new AutoPaths(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
//        r.overrideAutoEndPose(autoPaths.getPose(AutoPaths.PoseId.START_FRONT));
//        schedule(
//                new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                                new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.START_FRONT_TO_SHOOT_FRONT)),
//                                new SequentialCommandGroup(
//                                        new InstantCommand(() -> r.turret.setTarget(Turret.Target.MOTIF)),
//                                        new WaitCommand(400),
//                                        new InstantCommand(() -> r.ll.setMode(Limelight.Mode.MOTIF_DETECTION)),
//                                        new WaitUntilCommand(() -> MyRobot.currentMotif != null),
//                                        new ParallelCommandGroup(
//                                                new InstantCommand(() -> r.ll.setMode(Limelight.Mode.LOCALIZATION)),
//                                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL))
//                                        )
//                                ),
//                                r.door.setOpenCommand(true)
//                        ),
//                        r.shootAll().withTimeout(10000),
//                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_FRONT_TO_FRONT_SPIKE_START)),
//                        new InstantCommand(() -> r.f.setMaxPower(0.35)),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.FRONT_SPIKE_START_TO_FRONT_SPIKE_1)),
//                                        new WaitCommand(1200),
//                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.FRONT_SPIKE_1_TO_FRONT_SPIKE_2)),
//                                        new WaitCommand(1200),
//                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.FRONT_SPIKE_2_TO_FRONT_SPIKE_END)),
//                                        new WaitCommand(1200)
//                                ),
//                                new ParallelRaceGroup(r.intakeAll(), new WaitCommand(5000))
//                        ),
//                        new InstantCommand(() -> r.f.setMaxPower(0.8)),
//                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.FRONT_SPIKE_END_TO_GATE)),
//                        r.intake.setPowerInstant(Intake.IntakeMotorPowerConfig.STOP),
//                        new WaitCommand(2000),
//                        r.goTo(autoPaths.getPose(AutoPaths.PoseId.SHOOT_FRONT)),
//                        r.shootMotifSafe(),
//                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_FRONT_TO_MID_SPIKE_START)),
//                        new InstantCommand(() -> r.f.setMaxPower(0.35)),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.MID_SPIKE_START_TO_MID_SPIKE_1)),
//                                        new WaitCommand(1200),
//                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.MID_SPIKE_1_TO_MID_SPIKE_2)),
//                                        new WaitCommand(1200),
//                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.MID_SPIKE_2_TO_MID_SPIKE_END)),
//                                        new WaitCommand(1200)
//                                ),
//                                new ParallelRaceGroup(r.intakeAll(), new WaitCommand(5000))
//                        ),
//                        new InstantCommand(() -> r.f.setMaxPower(1)),
//                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.MID_SPIKE_END_TO_SHOOT_FRONT)),
//                        r.shootMotifSafe()
//                        /*,
//                        r.goTo(autoPaths.getPose(AutoPaths.PoseId.MID_SPIKE_START)),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new InstantCommand(() -> r.f.setMaxPower(0.3)),
//                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.MID_SPIKE_START_TO_MID_SPIKE_END)),
//                                        new InstantCommand(() -> r.f.setMaxPower(1))
//                                ),
//                                new ParallelRaceGroup(r.intakeAll(), new WaitCommand(5000))
//                        ),
//                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.MID_SPIKE_END_TO_SHOOT_FRONT)),
//                        r.shootMotifSafe()
//                         */
//                )
//        );
//    }
//}
