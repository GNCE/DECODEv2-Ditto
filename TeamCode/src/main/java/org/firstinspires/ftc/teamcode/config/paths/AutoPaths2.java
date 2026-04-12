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

public final class AutoPaths2 {
    // ====== POSES (BLUE ONLY) ======

    // POSE FOR RED
    // START_FRONT (new Pose(25.34814419, 134, Math.toRadians(-127.879678))),

    static final double X_OFFSET_FRONT = -7.414;
    static final double Y_OFFSET_FRONT = -1.617463638;
    // 5.78, 9.23
    // 5.45, 9.485
    // 133.95 6.866
    // 120.348, 132.2373, -142.08185
    // 34.125 135.9, -90

    // 133.1 6.4271

    // 6.40419

    // Current Pose: (26.71078354208486, 134.28253636247763, -89.53565892394403)


    static double SINGLE_X_OFFSET = -1.5;

    public enum PoseId {
        // FRONT AUTO POSES
        START_FRONT (new Pose(34.125 + X_OFFSET_FRONT, 135.9+Y_OFFSET_FRONT, Math.toRadians(270))), // 30.2946, 133.37
        START_FRONT_SHOOT_1 (new Pose(35+ X_OFFSET_FRONT, 101.587+Y_OFFSET_FRONT, Math.toRadians(270))),

        SPIKE_FRONT_TRIPLE_START (new Pose(27.343+ X_OFFSET_FRONT, 101+Y_OFFSET_FRONT, Math.toRadians(270))),
        SPIKE_FRONT_TRIPLE_END (new Pose(27.343+ X_OFFSET_FRONT, 84+Y_OFFSET_FRONT, Math.toRadians(270))),
        START_FRONT_SHOOT_AFTER_FRONT_TRIPLE (new Pose(45.893+ X_OFFSET_FRONT, 92.425+Y_OFFSET_FRONT, Math.toRadians(270))),

        SPIKE_MID_TRIPLE_START (new Pose(26.3+ X_OFFSET_FRONT, 80+Y_OFFSET_FRONT, Math.toRadians(270))),
        SPIKE_MID_TRIPLE_END (new Pose(26.3+ X_OFFSET_FRONT, 62+Y_OFFSET_FRONT, Math.toRadians(270))),
        SPIKE_MID_THEN_GATE_OPEN (new Pose(24.8+ X_OFFSET_FRONT, 75.833+Y_OFFSET_FRONT, Math.toRadians(-145))),
        SPIKE_MID_THEN_SHOOT_FRONT (new Pose(59.82+ X_OFFSET_FRONT, 77.8+Y_OFFSET_FRONT, Math.toRadians(-142.34))),
        GATE_INTAKE_TRIPLE (new Pose(18.40+ X_OFFSET_FRONT, 60.15+Y_OFFSET_FRONT, Math.toRadians(152))), // 12.3 56.66 150.14
        GATE_INTAKE_SAFE_TRIPLE (new Pose(20.25+ X_OFFSET_FRONT, 55.25+Y_OFFSET_FRONT, Math.toRadians(151))),
        GATE_INTAKE_SAFE_TRIPLE_SAFE (new Pose(18+ X_OFFSET_FRONT, 57+Y_OFFSET_FRONT, Math.toRadians(180))),
        FRONT_SHOOT_AFTER_GATE (new Pose(63.268+ X_OFFSET_FRONT, 77.5655+Y_OFFSET_FRONT, Math.toRadians(215))),
        FRONT_SHOOT_AFTER_GATE_FINAL (new Pose(63.268+ X_OFFSET_FRONT, 77.5655+Y_OFFSET_FRONT, Math.toRadians(260))),
        FRONT_SHOOT_AFTER_GATE_FINAL_FINAL (new Pose(63.268+ X_OFFSET_FRONT, 77.5655+Y_OFFSET_FRONT, Math.toRadians(160))),

        // SINGLE AUTO CLOSE
        SINGLE_FRONT_START (new Pose(17, 111.64, Math.toRadians(180))),
        SINGLE_MID_SPIKE_START (new Pose(58, 83, Math.toRadians(206))),
        SINGLE_MID_SPIKE_CONTROL (new Pose(60, 55.5)),
        SINGLE_MID_SPIKE_END(new Pose(9, 59, Math.toRadians(180))),
        SINGLE_CLOSE_SPIKE_START (new Pose(42.578, 83.5327, Math.toRadians(180))),
        SINGLE_CLOSE_SPIKE_END (new Pose(16.5, 84, Math.toRadians(180))),
        SINGLE_FINAL_SHOOT (new Pose(59, 100, Math.toRadians(200))),


        SHOOT_FRONT_GATE_CYCLE (new Pose(55, 80, Math.toRadians(222))),
        GATE_INTAKE (new Pose(15.76, 60.02)),
        GATE_INTAKE_SAFE (new Pose(15.3, 57)),
        SHOOT_FRONT_TO_GATE_INTAKE_CONTROL (new Pose(34.41388518024032, 56.13885180240321)),
        GATE_INTAKE_SAFE_TO_SHOOT_FRONT_CONTROL (new Pose(34.41388518024032, 56.13885180240321)),
        // 17.348, 63.94, 151.96

        // BACK AUTO POSES
        START_BACK  (new Pose(53.825, 7.984, Math.toRadians(180))), // TODO
        HP_END (new Pose(11, 8.969, Math.toRadians(180))),
        SHOOT_BACK_1 (new Pose(57.3185, 20.1238, Math.toRadians(155))),
        TRIPLE_FAR_SPIKE_CONTROL (new Pose(57, 37)),
        TRIPLE_FAR_SPIKE_END(new Pose(10, 35.8, Math.toRadians(180))),
        SHOOT_BACK_2 (new Pose(50, 8.969, Math.toRadians(180))),
        SHOOT_BACK_3 (new Pose(50.386, 13.819, Math.toRadians(151.526))),
        SHOOT_FINAL (new Pose(53, 16, Math.toRadians(180))),
        PARK_FINAL (new Pose(46, 16, Math.toRadians(180))),
        TRIPLE_GATE_SWEEP_END(new Pose(11, 36, Math.toRadians(103.421))),
        TRIPLE_GATE_SWEEP_CONTROL (new Pose(24, 12)),
        ALT_GATE_SWEEP_MID (new Pose(19, 10, Math.toRadians(140))),
        ALT_GATE_SWEEP_END (new Pose(13, 30, Math.toRadians(140))),


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

        INTERMEDIATE_FAR_INTAKE_POSE   (new Pose(54.5, 29.5)),
        FAR_SPIKE_END     (new Pose(13, 36, Math.toRadians(180)));


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

        // CLOSE SINGLE AUTO
        SINGLE_CLOSE_START_TO_MID_SPIKE_START,
        SINGLE_MID_SPIKE_START_TO_MID_SPIKE_END,
        SINGLE_MID_SPIKE_END_TO_FRONT_SHOOT_AFTER_GATE,
        SINGLE_SHOOT_AFTER_GATE_TO_CLOSE_SPIKE_START,
        SINGLE_SHOOT_AFTER_GATE_TO_CLOSE_SPIKE_END,
        SINGLE_SHOOT_AFTER_GATE_TO_FAR_SPIKE_END,
        SINGLE_CLOSE_SPIKE_START_TO_CLOSE_SPIKE_END,
        SINGLE_CLOSE_SPIKE_END_TO_CLOSE_FINAL_SHOOT,
        SINGLE_FAR_SPIKE_END_TO_FRONT_SHOOT_AFTER_GATE,

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
        GATE_SWEEP_END_TO_SHOOT_BACK_3,
        GATE_SWEEP_END_TO_SHOOT_BACK_2,
        HP_END_TO_SHOOT_BACK_2,

        SHOOT_BACK_2_ALT_GATE_SWEEP_MID,
        SHOOT_BACK_2_ALT_GATE_SWEEP_END,
        ALT_GATE_SWEEP_MID_TO_GATE_SWEEP_END,
        ALT_GATE_SWEEP_END_TO_SHOOT_BACK_2,
        TRIPLE_GATE_SAFE_TO_GATE_INTAKE_SAFE_SAFE,
        GATE_INTAKE_SAFE_SAFE_TO_SHOOT,
        GATE_INTAKE_SAFE_SAFE_TO_SHOOT_FINAl,
    }

    // ====== INSTANCE STATE ======
    private final Follower f;
    private final Alliance a;

    private final EnumMap<PoseId, Pose> p     = new EnumMap<>(PoseId.class);
    private final EnumMap<PathId, PathChain> paths = new EnumMap<>(PathId.class);
    public AutoPaths2(Follower f, Alliance a) {
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
        // CLOSE SINGLE AUTO
        put(PathId.SINGLE_CLOSE_START_TO_MID_SPIKE_START, PoseId.SINGLE_FRONT_START, PoseId.SINGLE_MID_SPIKE_START);


        paths.put(PathId.SINGLE_MID_SPIKE_START_TO_MID_SPIKE_END,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.SINGLE_MID_SPIKE_START), getPose(PoseId.SINGLE_MID_SPIKE_CONTROL), getPose(PoseId.SINGLE_MID_SPIKE_END)))
                        .setLinearHeadingInterpolation(getPose(PoseId.SINGLE_MID_SPIKE_START).getHeading(), getPose(PoseId.SINGLE_MID_SPIKE_END).getHeading())
                        .build()
        );

        paths.put(PathId.SINGLE_MID_SPIKE_END_TO_FRONT_SHOOT_AFTER_GATE,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.SINGLE_MID_SPIKE_END), getPose(PoseId.SINGLE_MID_SPIKE_CONTROL), getPose(PoseId.FRONT_SHOOT_AFTER_GATE)))
                        .setLinearHeadingInterpolation(getPose(PoseId.SINGLE_MID_SPIKE_END).getHeading(), getPose(PoseId.FRONT_SHOOT_AFTER_GATE).getHeading())
                        .build()
        );

        put(PathId.SINGLE_SHOOT_AFTER_GATE_TO_CLOSE_SPIKE_START, PoseId.FRONT_SHOOT_AFTER_GATE, PoseId.SINGLE_CLOSE_SPIKE_START);
        put(PathId.SINGLE_CLOSE_SPIKE_START_TO_CLOSE_SPIKE_END, PoseId.SINGLE_CLOSE_SPIKE_START, PoseId.SINGLE_CLOSE_SPIKE_END);
        put(PathId.SINGLE_CLOSE_SPIKE_END_TO_CLOSE_FINAL_SHOOT, PoseId.SINGLE_CLOSE_SPIKE_END, PoseId.SINGLE_FINAL_SHOOT);

        paths.put(PathId.SINGLE_SHOOT_AFTER_GATE_TO_CLOSE_SPIKE_END,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL_FINAL), getPose(PoseId.SINGLE_CLOSE_SPIKE_START), getPose(PoseId.SINGLE_CLOSE_SPIKE_END)))
                        .setLinearHeadingInterpolation(getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL_FINAL).getHeading(), getPose(PoseId.SINGLE_CLOSE_SPIKE_END).getHeading())
                        .build()
        );

        paths.put(PathId.SINGLE_FAR_SPIKE_END_TO_FRONT_SHOOT_AFTER_GATE,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.FAR_SPIKE_END), getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL_FINAL)))
                        .setLinearHeadingInterpolation(getPose(PoseId.FAR_SPIKE_END).getHeading(), getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL_FINAL).getHeading())
                        .build()
        );

        paths.put(PathId.SINGLE_SHOOT_AFTER_GATE_TO_FAR_SPIKE_END,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL), getPose(PoseId.INTERMEDIATE_FAR_INTAKE_POSE), getPose(PoseId.FAR_SPIKE_END)))
                        .setLinearHeadingInterpolation(getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL).getHeading(), getPose(PoseId.FAR_SPIKE_END).getHeading())
                        .build()
        );

        // FAR TRIPLE AUTO
        put(PathId.START_BACK_TO_HUMAN_PLAYER_END, PoseId.START_BACK, PoseId.HP_END);
        put(PathId.HP_TO_SHOOT_BACK_1, PoseId.HP_END, PoseId.SHOOT_BACK_1);
        paths.put(PathId.SHOOT_BACK_1_TO_FAR_SPIKE_END,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.SHOOT_BACK_1), getPose(PoseId.TRIPLE_FAR_SPIKE_CONTROL), getPose(PoseId.TRIPLE_FAR_SPIKE_END)))
                        .setLinearHeadingInterpolation(getPose(PoseId.SHOOT_BACK_1).getHeading(), getPose(PoseId.TRIPLE_FAR_SPIKE_END).getHeading())
                        .build()
        );

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
        put(PathId.GATE_SWEEP_END_TO_SHOOT_BACK_2, PoseId.TRIPLE_GATE_SWEEP_END, PoseId.SHOOT_BACK_2);
        put(PathId.HP_END_TO_SHOOT_BACK_2, PoseId.HP_END, PoseId.SHOOT_BACK_2);


        paths.put(PathId.SHOOT_BACK_2_ALT_GATE_SWEEP_END,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.SHOOT_BACK_2), getPose(PoseId.ALT_GATE_SWEEP_MID)))
                        .setConstantHeadingInterpolation(getPose(PoseId.SHOOT_BACK_2).getHeading())
                        .addPath(new BezierLine(getPose(PoseId.ALT_GATE_SWEEP_MID), getPose(PoseId.ALT_GATE_SWEEP_END)))
                        .setConstantHeadingInterpolation(getPose(PoseId.ALT_GATE_SWEEP_END).getHeading())
                        .setGlobalDeceleration()
                        .build()
        );
        put(PathId.SHOOT_BACK_2_ALT_GATE_SWEEP_MID, PoseId.SHOOT_BACK_2, PoseId.ALT_GATE_SWEEP_MID);
        put(PathId.ALT_GATE_SWEEP_MID_TO_GATE_SWEEP_END, PoseId.ALT_GATE_SWEEP_MID, PoseId.ALT_GATE_SWEEP_END);
        put(PathId.ALT_GATE_SWEEP_END_TO_SHOOT_BACK_2, PoseId.ALT_GATE_SWEEP_END, PoseId.SHOOT_BACK_2);



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

        paths.put(PathId.TRIPLE_GATE_SAFE_TO_GATE_INTAKE_SAFE_SAFE,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE), getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE_SAFE)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.3, getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE).getHeading(), getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE_SAFE).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.3, 1, getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE).getHeading(), getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE_SAFE).getHeading())
                        ))
                        .build()
        );

        paths.put(PathId.GATE_INTAKE_SAFE_SAFE_TO_SHOOT,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE_SAFE), getPose(PoseId.FRONT_SHOOT_AFTER_GATE)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.3, getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE_SAFE).getHeading(), getPose(PoseId.FRONT_SHOOT_AFTER_GATE).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.3, 1, getPose(PoseId.FRONT_SHOOT_AFTER_GATE).getHeading(), getPose(PoseId.FRONT_SHOOT_AFTER_GATE).getHeading())
                        ))
                        .build()
        );

        paths.put(PathId.GATE_INTAKE_SAFE_SAFE_TO_SHOOT_FINAl,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE_SAFE), getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.3, getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE_SAFE).getHeading(), getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.3, 1, getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL).getHeading(), getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL).getHeading())
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
