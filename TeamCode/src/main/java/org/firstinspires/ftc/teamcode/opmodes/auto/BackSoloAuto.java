//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//import com.seattlesolvers.solverslib.command.WaitUntilCommand;
//
//import org.firstinspires.ftc.teamcode.config.commands.FollowPathCommand;
//import org.firstinspires.ftc.teamcode.config.core.MyRobot;
//import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
//import org.firstinspires.ftc.teamcode.config.core.util.Artifact;
//import org.firstinspires.ftc.teamcode.config.core.util.ArtifactMatch;
//import org.firstinspires.ftc.teamcode.config.core.util.OpModeType;
//import org.firstinspires.ftc.teamcode.config.paths.AutoPaths;
//import org.firstinspires.ftc.teamcode.config.subsystems.Limelight;
//import org.firstinspires.ftc.teamcode.config.subsystems.Turret;
//
//@Autonomous(group="Solo Auto", name="Back Solo Auto")
//public class BackSoloAuto extends MyCommandOpMode {
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
//        r.overrideAutoEndPose(autoPaths.getPose(AutoPaths.PoseId.START_BACK));
//        schedule(
//                new SequentialCommandGroup(
//                        new SequentialCommandGroup(
//                                new ParallelCommandGroup(
//                                        r.goTo(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK)),
//                                        new SequentialCommandGroup(
//                                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.MOTIF)),
//                                                new WaitCommand(450),
//                                                new InstantCommand(() -> r.ll.setMode(Limelight.Mode.MOTIF_DETECTION)),
//                                                new WaitUntilCommand(() -> MyRobot.currentMotif != null),
//                                                new ParallelCommandGroup(
//                                                        new InstantCommand(() -> r.ll.setMode(Limelight.Mode.LOCALIZATION)),
//                                                        new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL))
//                                                )
//                                        )
//                                ),
//                                r.shootMotifSafe(),
//                                r.intake.setPowerInstant(0),
//                                r.goTo(autoPaths.getPose(AutoPaths.PoseId.HUMAN_PLAYER_ZONE_1)),
//                                new WaitCommand(800),
//                                new ParallelCommandGroup(
//                                        new SequentialCommandGroup(
//                                                new InstantCommand(() -> r.f.setMaxPower(0.4)),
//                                                r.goTo(autoPaths.getPose(AutoPaths.PoseId.HUMAN_PLAYER_ZONE_END)),
//                                                new InstantCommand(() -> r.f.setMaxPower(1))
//                                        ),
//                                        new SequentialCommandGroup(
//                                                r.intakeForcedWithTimeout(1200, Artifact.PURPLE),
//                                                r.intakeForcedWithTimeout(700, Artifact.GREEN),
//                                                r.intakeForcedWithTimeout(700, Artifact.PURPLE)
//                                        )
//                                ),
//                                r.goTo(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK)),
//                                r.shootMotifSafe(),
//                                r.goTo(autoPaths.getPose(AutoPaths.PoseId.MID_SPIKE_START)),
//                                new WaitCommand(1000),
//                                new ParallelCommandGroup(
//                                        new SequentialCommandGroup(
//                                                new InstantCommand(() -> r.f.setMaxPower(0.3)),
//                                                new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.MID_SPIKE_START_TO_MID_SPIKE_END)),
//                                                new InstantCommand(() -> r.f.setMaxPower(1))
//                                        ),
//                                        new SequentialCommandGroup(
//                                                r.intakeForcedWithTimeout(1200, Artifact.PURPLE),
//                                                r.intakeForcedWithTimeout(700, Artifact.GREEN),
//                                                r.intakeForcedWithTimeout(700, Artifact.PURPLE)
//                                        )
//                                ),
//                                new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.MID_SPIKE_END_TO_SHOOT_FRONT)),
//                                r.shootMotifSafe()
//                        ).withTimeout(29500),
//                        r.goTo(autoPaths.getPose(AutoPaths.PoseId.FRONT_PARK))
//                )
//        );
//    }
//}
