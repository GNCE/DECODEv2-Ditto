package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Alliance;

import java.util.EnumMap;

public final class AutoPaths {
    // ====== POSES (BLUE ONLY) ======

    // POSE FOR RED
    // START_FRONT (new Pose(25.34814419, 134, Math.toRadians(-127.879678))),
    public enum PoseId {
        START_FRONT (new Pose(28.09, 130.592, Math.toRadians(-124.9))),
        START_BACK  (new Pose(48 + MyRobot.chassisBack, MyRobot.chassisLeft, Math.toRadians(180))), // TODO

        SHOOT_FRONT (new Pose(36, 108, Math.toRadians(180))),
        SHOOT_FRONT_270 (new Pose(36, 108, Math.toRadians(270))),
        SHOOT_FRONT_90 (new Pose(36, 108, Math.toRadians(90))),
        SHOOT_BACK  (new Pose(63.63684913217624, 24.224299065420563, Math.toRadians(180))),
        GATE   (new Pose(23.8, 68, Math.toRadians(180))), // TODO: CHECK
        GATE_BALLS (new Pose(24.4, 64, Math.toRadians(180))),
        SHOOT_FRONT_TO_GATE_CONTROL (new Pose(45.18024032042723, 72.09612817089453, Math.toRadians(180))),
        GATE_SAFE_INTAKE (new Pose(20, 55, Math.toRadians(120))),
        GATE_TO_GATE_SAFE_INTAKE_CONTROL (new Pose(23.455273698264353, 49.98664886515355, Math.toRadians(180))),

        GATE_SAFE_INTAKE_TO_SHOOT_FRONT_90_CONTROL (new Pose(49.025367156208276, 60.75300400534044, Math.toRadians(180))),

        FRONT_SPIKE_START (new Pose(50, 81.5, Math.toRadians(180))), // RED 82
        FRONT_SPIKE_1 (new Pose(37, 81.5, Math.toRadians(180))),
        FRONT_SPIKE_2 (new Pose(31.8, 81.5, Math.toRadians(180))),
        FRONT_SPIKE_END   (new Pose(26, 81.5, Math.toRadians(180))),
        FRONT_SPIKE_END_TO_GATE_CONTROL (new Pose(50, 75)),



        MID_SPIKE_START   (new Pose(52, 58, Math.toRadians(180))), // 58
        MID_SPIKE_1 (new Pose(37, 58, Math.toRadians(180))),
        MID_SPIKE_2 (new Pose(31.8, 58, Math.toRadians(180))),
        GATE_TO_MID_START_CONTROL (new Pose(40, 74.5)),
        MID_SPIKE_END     (new Pose(22, 58, Math.toRadians(180))),

        MID_SPIKE_END_TO_FRONT_SHOOT_CONTROL (new Pose(45.75700934579439, 57.292389853137514)),

        FAR_SPIKE_START   (new Pose(50, 34, Math.toRadians(180))),
        FAR_SPIKE_END     (new Pose(23, 34, Math.toRadians(180))),
        FAR_SPIKE_END_TO_FRONT_SHOOT_CONTROL (new Pose(47.67957276368491, 45.37249666221629, Math.toRadians(180))),

        HUMAN_PLAYER_ZONE_1(new Pose(14.034712950600802, 18.64886515353805, Math.toRadians(180))),
        HUMAN_PLAYER_ZONE_2(new Pose(14.034712950600802, 12.688918558077429, Math.toRadians(180))),

        HUMAN_PLAYER_ZONE_END (new Pose(14.034712950600802, 7.882510013351133, Math.toRadians(180))),

        FRONT_PARK (new Pose(55, 117, Math.toRadians(180))),
        BACK_PARK (new Pose(61.90654205607476, 34.413885180240314, Math.toRadians(180)));

        private final Pose blue;

        PoseId(Pose blue) {
            this.blue = blue;
        }

        public Pose forAlliance(Alliance a) {
            return (a == Alliance.RED) ? blue.mirror() : blue;
        }
    }

    /*
    22.439522, 129.510988 -126.8238404
    114.4188107, 122.0312788355069, -49.40561
     */

    // ====== EVERY EDGE IS A PATH ======
    public enum PathId {
        START_FRONT_TO_SHOOT_FRONT,

        SHOOT_FRONT_TO_FRONT_SPIKE_START,
        FRONT_SPIKE_START_TO_FRONT_SPIKE_END,
        FRONT_SPIKE_START_TO_FRONT_SPIKE_1,
        FRONT_SPIKE_1_TO_FRONT_SPIKE_2,
        FRONT_SPIKE_2_TO_FRONT_SPIKE_END,
        FRONT_SPIKE_END_TO_SHOOT_FRONT,
        FRONT_SPIKE_END_TO_GATE,
        FRONT_SHOOT_TO_GATE,
        FRONT_SHOOT_TO_GATE_BALLS,

        SHOOT_FRONT_TO_MID_SPIKE_START,
        MID_SPIKE_START_TO_MID_SPIKE_END,
        MID_SPIKE_START_TO_MID_SPIKE_1,
        MID_SPIKE_1_TO_MID_SPIKE_2,
        MID_SPIKE_2_TO_MID_SPIKE_END,
        MID_SPIKE_END_TO_SHOOT_FRONT,

        SHOOT_FRONT_TO_FAR_SPIKE_START,
        FAR_SPIKE_START_TO_FAR_SPIKE_END,
        FAR_SPIKE_END_TO_SHOOT_FRONT,
        FAR_SPIKE_END_TO_SHOOT_FRONT_270,
        GATE_TO_MID_SPIKE_START,
        GATE_TO_GATE_SAFE_INTAKE,
        GATE_SAFE_INTAKE_TO_SHOOT_FRONT
    }

    // ====== INSTANCE STATE ======
    private final Follower f;
    private final Alliance a;

    private final EnumMap<PoseId, Pose> p     = new EnumMap<>(PoseId.class);
    private final EnumMap<PathId, PathChain> paths = new EnumMap<>(PathId.class);

    // ====== CONSTRUCTOR: BUILD EVERYTHING ======
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

        // START → SHOOT
        put(PathId.START_FRONT_TO_SHOOT_FRONT,
                PoseId.START_FRONT, PoseId.SHOOT_FRONT);

        // FRONT SPIKE
        put(PathId.SHOOT_FRONT_TO_FRONT_SPIKE_START,
                PoseId.SHOOT_FRONT, PoseId.FRONT_SPIKE_START);
        paths.put(
                PathId.FRONT_SPIKE_START_TO_FRONT_SPIKE_END,
                line(PoseId.FRONT_SPIKE_START, PoseId.FRONT_SPIKE_END).build()
                );
        // put(PathId.FRONT_SPIKE_START_TO_FRONT_SPIKE_END, PoseId.FRONT_SPIKE_START, PoseId.FRONT_SPIKE_END);
        put(PathId.FRONT_SPIKE_END_TO_SHOOT_FRONT,
                PoseId.FRONT_SPIKE_END, PoseId.SHOOT_FRONT);

        paths.put(PathId.FRONT_SPIKE_END_TO_GATE,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.FRONT_SPIKE_END), getPose(PoseId.FRONT_SPIKE_END_TO_GATE_CONTROL), getPose(PoseId.GATE)))
                        .setLinearHeadingInterpolation(getPose(PoseId.FRONT_SPIKE_END).getHeading(), getPose(PoseId.GATE).getHeading())
                        .build()
        );

        paths.put(PathId.FRONT_SHOOT_TO_GATE,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.SHOOT_FRONT), getPose(PoseId.SHOOT_FRONT_TO_GATE_CONTROL), getPose(PoseId.GATE)))
                        .setLinearHeadingInterpolation(getPose(PoseId.SHOOT_FRONT).getHeading(), getPose(PoseId.GATE).getHeading())
                        .build()
        );

        paths.put(PathId.FRONT_SHOOT_TO_GATE_BALLS,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.SHOOT_FRONT), getPose(PoseId.SHOOT_FRONT_TO_GATE_CONTROL), getPose(PoseId.GATE_BALLS)))
                        .setLinearHeadingInterpolation(getPose(PoseId.SHOOT_FRONT).getHeading(), getPose(PoseId.GATE_BALLS).getHeading())
                        .setTimeoutConstraint(50)
                        .build()
        );

        paths.put(PathId.GATE_SAFE_INTAKE_TO_SHOOT_FRONT,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.GATE_SAFE_INTAKE), getPose(PoseId.GATE_SAFE_INTAKE_TO_SHOOT_FRONT_90_CONTROL), getPose(PoseId.SHOOT_FRONT_90)))
                        .setLinearHeadingInterpolation(getPose(PoseId.GATE_SAFE_INTAKE).getHeading(), getPose(PoseId.SHOOT_FRONT_90).getHeading())
                        .build()
        );

        paths.put(PathId.GATE_TO_GATE_SAFE_INTAKE,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.GATE), getPose(PoseId.GATE_TO_GATE_SAFE_INTAKE_CONTROL), getPose(PoseId.GATE_SAFE_INTAKE)))
                        .setLinearHeadingInterpolation(getPose(PoseId.GATE).getHeading(), getPose(PoseId.GATE_SAFE_INTAKE).getHeading())
                        .build()
        );

        paths.put(PathId.FAR_SPIKE_END_TO_SHOOT_FRONT_270,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.FAR_SPIKE_END), getPose(PoseId.FAR_SPIKE_END_TO_FRONT_SHOOT_CONTROL), getPose(PoseId.SHOOT_FRONT_270)))
                        .setLinearHeadingInterpolation(getPose(PoseId.FAR_SPIKE_END).getHeading(), getPose(PoseId.SHOOT_FRONT_270).getHeading())
                        .build()
        );


        put(PathId.FRONT_SPIKE_START_TO_FRONT_SPIKE_1, PoseId.FRONT_SPIKE_START, PoseId.FRONT_SPIKE_1);
        put(PathId.FRONT_SPIKE_1_TO_FRONT_SPIKE_2, PoseId.FRONT_SPIKE_1, PoseId.FRONT_SPIKE_2);
        put(PathId.FRONT_SPIKE_2_TO_FRONT_SPIKE_END, PoseId.FRONT_SPIKE_2, PoseId.FRONT_SPIKE_END);

        put(PathId.MID_SPIKE_START_TO_MID_SPIKE_1, PoseId.MID_SPIKE_START, PoseId.MID_SPIKE_1);
        put(PathId.MID_SPIKE_1_TO_MID_SPIKE_2, PoseId.MID_SPIKE_1, PoseId.MID_SPIKE_2);
        put(PathId.MID_SPIKE_2_TO_MID_SPIKE_END, PoseId.MID_SPIKE_2, PoseId.MID_SPIKE_END);


        // MID SPIKE
        put(PathId.SHOOT_FRONT_TO_MID_SPIKE_START,
                PoseId.SHOOT_FRONT, PoseId.MID_SPIKE_START);

        put(PathId.MID_SPIKE_START_TO_MID_SPIKE_END,
                PoseId.MID_SPIKE_START, PoseId.MID_SPIKE_END);

        paths.put(PathId.MID_SPIKE_END_TO_SHOOT_FRONT,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.MID_SPIKE_END), getPose(PoseId.MID_SPIKE_END_TO_FRONT_SHOOT_CONTROL), getPose(PoseId.SHOOT_FRONT)))
                        .setLinearHeadingInterpolation(getPose(PoseId.MID_SPIKE_END).getHeading(), getPose(PoseId.SHOOT_FRONT).getHeading())
                        .build()
        );

        // FAR SPIKE
        put(PathId.SHOOT_FRONT_TO_FAR_SPIKE_START,
                PoseId.SHOOT_FRONT, PoseId.FAR_SPIKE_START);
        put(PathId.FAR_SPIKE_START_TO_FAR_SPIKE_END,
                PoseId.FAR_SPIKE_START, PoseId.FAR_SPIKE_END);
        put(PathId.FAR_SPIKE_END_TO_SHOOT_FRONT,
                PoseId.FAR_SPIKE_END, PoseId.SHOOT_FRONT);
        paths.put(PathId.GATE_TO_MID_SPIKE_START,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.GATE), getPose(PoseId.GATE_TO_MID_START_CONTROL), getPose(PoseId.MID_SPIKE_START)))
                        .setLinearHeadingInterpolation(getPose(PoseId.GATE).getHeading(), getPose(PoseId.MID_SPIKE_START).getHeading())
                        .build()
        );
    }

    // Small helper: ONE BezierLine = ONE PathChain
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
