package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.config.core.util.Alliance;

import java.util.EnumMap;

public final class AutoPaths {
    // ====== POSES (BLUE ONLY) ======
    public enum PoseId {
        START_FRONT (new Pose(28.081609, 132.511467, Math.toRadians(-127.879678))),
        START_BACK  (new Pose(54.69, 6.74, Math.toRadians(180))), // TODO

        SHOOT_FRONT (new Pose(40, 107, Math.toRadians(180))),
        SHOOT_BACK  (new Pose(54.69, 6.74, Math.toRadians(180))), // TODO
        GATE   (new Pose(19.4, 72.7, Math.toRadians(180))), // TODO: CHECK

        FRONT_SPIKE_START (new Pose(44, 84, Math.toRadians(180))),
        FRONT_SPIKE_1 (new Pose(37, 84, Math.toRadians(180))),
        FRONT_SPIKE_2 (new Pose(31.8, 84, Math.toRadians(180))),
        FRONT_SPIKE_END   (new Pose(19.5, 84, Math.toRadians(180))),
        FRONT_SPIKE_END_TO_GATE_CONTROL (new Pose(32.1068090787717, 80.55540720961282)),

        MID_SPIKE_START   (new Pose(42, 58, Math.toRadians(180))),
        MID_SPIKE_1 (new Pose(37, 58, Math.toRadians(180))),
        MID_SPIKE_2 (new Pose(31.8, 58, Math.toRadians(180))),
        GATE_TO_MID_START_CONTROL (new Pose(40, 74.5)),
        MID_SPIKE_END     (new Pose(9.5, 58, Math.toRadians(180))),

        MID_SPIKE_END_TO_FRONT_SHOOT_CONTROL (new Pose(45.75700934579439, 57.292389853137514)),

        FAR_SPIKE_START   (new Pose(47, 36, Math.toRadians(180))),
        FAR_SPIKE_END     (new Pose(20, 36, Math.toRadians(180))),

        FRONT_PARK (new Pose(55, 117, Math.toRadians(0)));

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

        SHOOT_FRONT_TO_MID_SPIKE_START,
        MID_SPIKE_START_TO_MID_SPIKE_END,
        MID_SPIKE_START_TO_MID_SPIKE_1,
        MID_SPIKE_1_TO_MID_SPIKE_2,
        MID_SPIKE_2_TO_MID_SPIKE_END,
        MID_SPIKE_END_TO_SHOOT_FRONT,

        SHOOT_FRONT_TO_FAR_SPIKE_START,
        FAR_SPIKE_START_TO_FAR_SPIKE_END,
        FAR_SPIKE_END_TO_SHOOT_FRONT,
        GATE_TO_MID_SPIKE_START
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
