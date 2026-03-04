package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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

    static final double START_FRONT_X = 33.1;

    public enum PoseId {
        // FRONT AUTO POSES
        START_FRONT (new Pose(START_FRONT_X, 135, Math.toRadians(-90))),
        START_FRONT_1 (new Pose(START_FRONT_X, 125, Math.toRadians(-90))),
        START_FRONT_1_1 (new Pose(34.79839786381842, 103.62616822429906)),
        START_FRONT_1_2 (new Pose(83, 75)),
        SHOOT_FRONT (new Pose(55, 80, Math.toRadians(180))),
        GATE_INTAKE (new Pose(15.76, 60.02)),
        SHOOT_FRONT_TO_GATE_INTAKE_CONTROL (new Pose(34.41388518024032, 56.13885180240321)),
 // 17.348, 63.94, 151.96

        // BACK AUTO POSES
        START_BACK  (new Pose(48 + MyRobot.chassisBack, MyRobot.chassisLeft, Math.toRadians(180))), // TODO


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

    /*
    22.439522, 129.510988 -126.8238404
    114.4188107, 122.0312788355069, -49.40561
     */

    // ====== EVERY EDGE IS A PATH ======
    public enum PathId {
        START_FRONT_TO_MID_SPIKE_END,
        MID_SPIKE_END_TO_SHOOT_FRONT,
        SHOOT_FRONT_TO_GATE_INTAKE,
        GATE_INTAKE_TO_SHOOT_FRONT,
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
        paths.put(PathId.START_FRONT_TO_MID_SPIKE_END,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.START_FRONT), getPose(PoseId.START_FRONT_1)))
                        .addPath(new BezierCurve(getPose(PoseId.START_FRONT_1), getPose(PoseId.START_FRONT_1_1), getPose(PoseId.START_FRONT_1_2), getPose(PoseId.MID_SPIKE_START)))
                        .addPath(new BezierLine(getPose(PoseId.MID_SPIKE_START), getPose(PoseId.MID_SPIKE_END)))
                        .setGlobalTangentHeadingInterpolation()
                        .setGlobalDeceleration()
                        .build()
        );
        paths.put(PathId.MID_SPIKE_END_TO_SHOOT_FRONT,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.MID_SPIKE_END), getPose(PoseId.SHOOT_FRONT)))
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build()
        );
        paths.put(PathId.SHOOT_FRONT_TO_GATE_INTAKE,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.SHOOT_FRONT), getPose(PoseId.SHOOT_FRONT_TO_GATE_INTAKE_CONTROL), getPose(PoseId.GATE_INTAKE)))
                        .setTangentHeadingInterpolation()
                        .setBrakingStrength(0.7)
                        .build()
        );
        paths.put(PathId.GATE_INTAKE_TO_SHOOT_FRONT,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.GATE_INTAKE), getPose(PoseId.SHOOT_FRONT_TO_GATE_INTAKE_CONTROL), getPose(PoseId.SHOOT_FRONT)))
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build()
        );
    }
 // 33.18, 134.21
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
