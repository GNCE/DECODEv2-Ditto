package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Alliance;

import java.util.EnumMap;

public final class AutoPaths {
    // ====== POSES (BLUE ONLY) ======

    // POSE FOR RED
    // START_FRONT (new Pose(25.34814419, 134, Math.toRadians(-127.879678))),

    static final double START_FRONT_X = 34.125;
    // 5.78, 9.23
    // 5.45, 9.485
    // 133.95 6.866
    // 120.348, 132.2373, -142.08185
    // 34.125 135.9, -90

    // 133.1 6.4271

    public enum PoseId {
        // FRONT AUTO POSES
        START_FRONT (new Pose(START_FRONT_X, 135.9, Math.toRadians(270))), // 30.2946, 133.37
        START_FRONT_SHOOT_1 (new Pose(34.48, 101.587, Math.toRadians(270))),

        SPIKE_FRONT_TRIPLE_START (new Pose(27.343, 100, Math.toRadians(270))),
        SPIKE_FRONT_TRIPLE_END (new Pose(27.343, 87, Math.toRadians(270))),
        START_FRONT_SHOOT_AFTER_FRONT_TRIPLE (new Pose(45.893, 92.425, Math.toRadians(270))),

        SPIKE_MID_TRIPLE_START (new Pose(26.3, 80, Math.toRadians(270))),
        SPIKE_MID_TRIPLE_END (new Pose(26.3, 64, Math.toRadians(270))),
        SPIKE_MID_THEN_GATE_OPEN (new Pose(25, 75.833, Math.toRadians(-145))),
        SPIKE_MID_THEN_SHOOT_FRONT (new Pose(59.82, 77.8, Math.toRadians(-142.34))),
        GATE_INTAKE_TRIPLE (new Pose(18, 61.2, Math.toRadians(150))), // 12.3 56.66 150.14
        GATE_INTAKE_SAFE_TRIPLE (new Pose(18, 57, Math.toRadians(150))),
        FRONT_SHOOT_AFTER_GATE (new Pose(63.268, 77.5655, Math.toRadians(210))),




        SHOOT_FRONT_GATE_CYCLE (new Pose(55, 80, Math.toRadians(222))),
        GATE_INTAKE (new Pose(15.76, 60.02)),
        GATE_INTAKE_SAFE (new Pose(15.76, 57)),
        SHOOT_FRONT_TO_GATE_INTAKE_CONTROL (new Pose(34.41388518024032, 56.13885180240321)),
        GATE_INTAKE_SAFE_TO_SHOOT_FRONT_CONTROL (new Pose(34.41388518024032, 56.13885180240321)),
 // 17.348, 63.94, 151.96

        // BACK AUTO POSES
        START_BACK  (new Pose(57.136, 8.969, Math.toRadians(180))), // TODO
        HP_END (new Pose(11, 8.969, Math.toRadians(180))),
        SHOOT_BACK_1 (new Pose(47.3185, 10.1238, Math.toRadians(136.376))),
        TRIPLE_FAR_SPIKE_END(new Pose(20.42, 36.542, Math.toRadians(136.376))),
        SHOOT_BACK_2 (new Pose(50, 8.969, Math.toRadians(180))),
        SHOOT_BACK_3 (new Pose(50.386, 13.819, Math.toRadians(151.526))),
        TRIPLE_GATE_SWEEP_END(new Pose(11, 36, Math.toRadians(103.421))),
        TRIPLE_GATE_SWEEP_CONTROL (new Pose(24, 12)),


        SHOOT_BACK (new Pose(56.5580774365821, 14.833110814419216)),

        SHOOT_BACK_TO_HUMAN_PLAYER_START_CONTROL_1 (new Pose(31.032710280373824, 42.33110814419225)),
        SHOOT_BACK_TO_HUMAN_PLAYER_START_CONTROL_2 (new Pose(17.593457943925234, 32.966622162883844)),
        SHOOT_BACK_TO_HUMAN_PLAYER_START (new Pose(8.35113484646195, 20.571428571428573)),
        HUMAN_PLAYER_END (new Pose(8.245660881174901, 6.862483311081451)),


        GATE_SWEEP_START(new Pose(9.805073431241656, 32.68357810413886)),
        GATE_SWEEP_END (new Pose(9.805073431241656, 48.79973297730308)),



        // SPIKE MARKS
        FRONT_SPIKE_START (new Pose(50, 81.5, Math.toRadians(180))), // RED 82
        FRONT_SPIKE_END   (new Pose(26, 81.5, Math.toRadians(180))),
        MID_SPIKE_START   (new Pose(48, 59, Math.toRadians(180))), // 58
        MID_SPIKE_END     (new Pose(22, 59, Math.toRadians(180))),

        FAR_SPIKE_START   (new Pose(50, 34, Math.toRadians(180))),
        FAR_SPIKE_END     (new Pose(23, 34, Math.toRadians(180)));



        private final Pose blue;

        PoseId(Pose blue) {
            this.blue = blue;
        }

        public Pose forAlliance(Alliance a) {
            return (a == Alliance.RED) ? blue.mirror() : blue;
        }
    }
    public enum PathId {
        START_FRONT_TO_MID_SPIKE_END,
        MID_SPIKE_END_TO_SHOOT_FRONT,
        SHOOT_FRONT_TO_GATE_INTAKE,
        GATE_INTAKE_TO_SHOOT_FRONT,
        GATE_INTAKE_TO_GATE_INTAKE_SAFE,
        GATE_INTAKE_SAFE_TO_SHOOT_FRONT,

        // CLOSE AUTO WITH TRIPLE
        START_FRONT_TO_SHOOT_FRONT_1,
        SHOOT_FRONT_1_TO_FRONT_TRIPLE_END,
        FRONT_TRIPLE_END_TO_SHOOT_FRONT,
        SHOOT_FRONT_TO_MID_TRIPLE_START,
        MID_TRIPLE_START_TO_MID_TRIPLE_END,
        MID_TRIPLE_END_TO_GATE_OPEN,
        GATE_OPEN_SIDE_TO_FRONT_SHOOT,
        SHOOT_TO_GATE_INTAKE_1,
        SHOOT_TO_GATE_INTAKE_NORMAL,
        GATE_INTAKE_TO_SHOOT_NORMAL,

        TRIPLE_GATE_INTAKE_TO_GATE_INTAKE_SAFE,
        TRIPLE_GATE_INTAKE_SAFE_TO_SHOOT,



        // FAR
        START_BACK_TO_HUMAN_PLAYER_END,
        HP_TO_SHOOT_BACK_1,
        SHOOT_BACK_1_TO_FAR_SPIKE_END,
        FAR_SPIKE_END_TO_SHOOT_BACK_2,
        SHOOT_BACK_2_TO_HP_END,
        HP_END_TO_SHOOT_BACK_3,
        SHOOT_BACK_3_TO_GATE_SWEEP_END,
        GATE_SWEEP_END_TO_SHOOT_BACK_3
    }

    // ====== INSTANCE STATE ======
    private final Follower f;
    private final Alliance a;

    private final EnumMap<PoseId, Pose> p     = new EnumMap<>(PoseId.class);
    private final EnumMap<PathId, PathChain> paths = new EnumMap<>(PathId.class);
    public AutoPaths(Follower f, Alliance a) {
        this.f = f;
        this.a = a;

        // Build alliance-correct poses
        for (PoseId id : PoseId.values()) {
            p.put(id, id.forAlliance(a));
        }

        // Build all individual path chains
        buildPaths();
    }

    // ====== ACCESS ======
    public Pose getPose(PoseId id) {
        return p.get(id);
    }

    public PathChain getPath(PathId id) {
        return paths.get(id);
    }

    // ====== PATH DEFINITIONS ======
    private void buildPaths() {
        // FAR TRIPLE AUTO
        put(PathId.START_BACK_TO_HUMAN_PLAYER_END, PoseId.START_BACK, PoseId.HP_END);
        put(PathId.HP_TO_SHOOT_BACK_1, PoseId.HP_END, PoseId.SHOOT_BACK_1);
        put(PathId.SHOOT_BACK_1_TO_FAR_SPIKE_END, PoseId.SHOOT_BACK_1, PoseId.TRIPLE_FAR_SPIKE_END);
        put(PathId.FAR_SPIKE_END_TO_SHOOT_BACK_2, PoseId.FAR_SPIKE_END, PoseId.SHOOT_BACK_2);
        put(PathId.SHOOT_BACK_2_TO_HP_END, PoseId.SHOOT_BACK_2, PoseId.HP_END);
        put(PathId.HP_END_TO_SHOOT_BACK_3, PoseId.HP_END, PoseId.SHOOT_BACK_3);
        paths.put(PathId.SHOOT_BACK_3_TO_GATE_SWEEP_END,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.SHOOT_BACK_3), getPose(PoseId.TRIPLE_GATE_SWEEP_CONTROL), getPose(PoseId.TRIPLE_GATE_SWEEP_END)))
                        .setTangentHeadingInterpolation()
                        .build()
        );
        paths.put(PathId.GATE_SWEEP_END_TO_SHOOT_BACK_3,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.TRIPLE_GATE_SWEEP_END), getPose(PoseId.SHOOT_BACK_3)))
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build()
        );

        // CLOSE TRIPLE AUTO
        put(PathId.START_FRONT_TO_SHOOT_FRONT_1, PoseId.START_FRONT, PoseId.START_FRONT_SHOOT_1);
        paths.put(PathId.SHOOT_FRONT_1_TO_FRONT_TRIPLE_END,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.START_FRONT_SHOOT_1), getPose(PoseId.SPIKE_FRONT_TRIPLE_START)))
                        .addPath(new BezierLine(getPose(PoseId.SPIKE_FRONT_TRIPLE_START), getPose(PoseId.SPIKE_FRONT_TRIPLE_END)))
                        .setGlobalConstantHeadingInterpolation(getPose(PoseId.START_FRONT_SHOOT_1).getHeading())
                        .setGlobalDeceleration()
                        .build()
        );
        put(PathId.FRONT_TRIPLE_END_TO_SHOOT_FRONT, PoseId.SPIKE_FRONT_TRIPLE_END, PoseId.START_FRONT_SHOOT_AFTER_FRONT_TRIPLE);
        put(PathId.SHOOT_FRONT_TO_MID_TRIPLE_START, PoseId.START_FRONT_SHOOT_AFTER_FRONT_TRIPLE, PoseId.SPIKE_MID_TRIPLE_START);
        put(PathId.MID_TRIPLE_START_TO_MID_TRIPLE_END, PoseId.SPIKE_MID_TRIPLE_START, PoseId.SPIKE_MID_TRIPLE_END);
        put(PathId.MID_TRIPLE_END_TO_GATE_OPEN, PoseId.SPIKE_MID_TRIPLE_END, PoseId.SPIKE_MID_THEN_GATE_OPEN);
        put(PathId.GATE_OPEN_SIDE_TO_FRONT_SHOOT, PoseId.SPIKE_MID_THEN_GATE_OPEN, PoseId.SPIKE_MID_THEN_SHOOT_FRONT);
        paths.put(PathId.SHOOT_TO_GATE_INTAKE_1,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.SPIKE_MID_THEN_SHOOT_FRONT), getPose(PoseId.GATE_INTAKE_TRIPLE)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.7, getPose(PoseId.SPIKE_MID_THEN_SHOOT_FRONT).getHeading(), getPose(PoseId.GATE_INTAKE_TRIPLE).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.7, 1, getPose(PoseId.GATE_INTAKE_TRIPLE).getHeading(), getPose(PoseId.GATE_INTAKE_TRIPLE).getHeading())
                        ))
                        .build()
        );
        paths.put(PathId.GATE_INTAKE_TO_SHOOT_NORMAL,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_TRIPLE), getPose(PoseId.FRONT_SHOOT_AFTER_GATE)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.3, getPose(PoseId.GATE_INTAKE_TRIPLE).getHeading(), getPose(PoseId.GATE_INTAKE_TRIPLE).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.3, 1, getPose(PoseId.GATE_INTAKE_TRIPLE).getHeading(), getPose(PoseId.FRONT_SHOOT_AFTER_GATE).getHeading())
                        ))
                        .build()
        );
        paths.put(PathId.SHOOT_TO_GATE_INTAKE_NORMAL,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.FRONT_SHOOT_AFTER_GATE), getPose(PoseId.GATE_INTAKE_TRIPLE)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.5, getPose(PoseId.FRONT_SHOOT_AFTER_GATE).getHeading(), getPose(PoseId.GATE_INTAKE_TRIPLE).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.5, 1, getPose(PoseId.GATE_INTAKE_TRIPLE).getHeading(), getPose(PoseId.GATE_INTAKE_TRIPLE).getHeading())
                        ))
                        .build()
        );
        put(PathId.TRIPLE_GATE_INTAKE_TO_GATE_INTAKE_SAFE, PoseId.GATE_INTAKE_TRIPLE, PoseId.GATE_INTAKE_SAFE_TRIPLE);
        paths.put(PathId.TRIPLE_GATE_INTAKE_SAFE_TO_SHOOT,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE), getPose(PoseId.FRONT_SHOOT_AFTER_GATE)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.3, getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE).getHeading(), getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.3, 1, getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE).getHeading(), getPose(PoseId.FRONT_SHOOT_AFTER_GATE).getHeading())
                        ))
                        .build()
        );

        paths.put(PathId.MID_SPIKE_END_TO_SHOOT_FRONT,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.MID_SPIKE_END), getPose(PoseId.SHOOT_FRONT_GATE_CYCLE)))
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build()
        );
        paths.put(PathId.SHOOT_FRONT_TO_GATE_INTAKE,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.SHOOT_FRONT_GATE_CYCLE), getPose(PoseId.SHOOT_FRONT_TO_GATE_INTAKE_CONTROL), getPose(PoseId.GATE_INTAKE)))
                        .setLinearHeadingInterpolation(getPose(PoseId.SHOOT_FRONT_GATE_CYCLE).getHeading(), getPose(PoseId.GATE_INTAKE).getHeading())
                        .build()
        );
        paths.put(PathId.GATE_INTAKE_TO_SHOOT_FRONT,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.GATE_INTAKE), getPose(PoseId.SHOOT_FRONT_TO_GATE_INTAKE_CONTROL), getPose(PoseId.SHOOT_FRONT_GATE_CYCLE)))
                        .setLinearHeadingInterpolation(getPose(PoseId.GATE_INTAKE).getHeading(), getPose(PoseId.SHOOT_FRONT_GATE_CYCLE).getHeading())
                        .build()
        );
        paths.put(PathId.GATE_INTAKE_TO_GATE_INTAKE_SAFE,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE), getPose(PoseId.GATE_INTAKE_SAFE)))
                        .setConstantHeadingInterpolation(getPose(PoseId.GATE_INTAKE).getHeading())
                        .build()
        );
        paths.put(PathId.GATE_INTAKE_SAFE_TO_SHOOT_FRONT,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.GATE_INTAKE_SAFE), getPose(PoseId.GATE_INTAKE_SAFE_TO_SHOOT_FRONT_CONTROL), getPose(PoseId.SHOOT_FRONT_GATE_CYCLE)))
                        .setLinearHeadingInterpolation(getPose(PoseId.GATE_INTAKE_SAFE).getHeading(), getPose(PoseId.GATE_INTAKE_SAFE_TO_SHOOT_FRONT_CONTROL).getHeading())
                        .build()
        );
    }
 // 33.18, 134.21
    private void put(PathId id, PoseId from, PoseId to) {
        paths.put(id,
                line(from, to).build()
        );
    }
    private PathBuilder line(PoseId from, PoseId to){
        Pose s = p.get(from);
        Pose e = p.get(to);
        return f.pathBuilder()
                .addPath(new BezierLine(s, e))
                .setLinearHeadingInterpolation(
                        s.getHeading(),
                        e.getHeading()
                );
    }
}
