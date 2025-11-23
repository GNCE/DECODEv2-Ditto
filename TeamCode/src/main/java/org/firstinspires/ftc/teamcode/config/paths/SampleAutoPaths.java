package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.core.util.Alliance;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.Function;

public final class SampleAutoPaths {
    public enum PoseId {
        START        (new Pose(12, 60, Math.toRadians(270))),
        LEFT_SPIKE   (new Pose(18, 36, Math.toRadians(250))),
        CENTER_SPIKE (new Pose(24, 36, Math.toRadians(270))),
        RIGHT_SPIKE  (new Pose(30, 36, Math.toRadians(290))),
        PARK         (new Pose(48, 12, Math.toRadians(270)));

        private final Pose bluePose;

        PoseId(Pose bluePose) {
            this.bluePose = bluePose;
        }

        public Pose forAlliance(Alliance alliance) {
            return (alliance == Alliance.RED)
                    ? bluePose.mirror()
                    : bluePose;
        }
    }
    public enum PathId {

        LEFT_SPIKE((f, p) ->
                f.pathBuilder()
                        .addPath(new BezierLine(p.apply(PoseId.START), p.apply(PoseId.LEFT_SPIKE)))
                        .build()
        );
        private final PathBuilder builder;

        PathId(PathBuilder builder) {
            this.builder = builder;
        }

        PathChain build(Follower follower, Function<PoseId, Pose> poseResolver) {
            return builder.build(follower, poseResolver);
        }

        @FunctionalInterface
        public interface PathBuilder {
            PathChain build(Follower follower, Function<PoseId, Pose> poseResolver);
        }
    }

    private final Follower f;
    private final Alliance a;
    private final EnumMap<PoseId, Pose> poseMap = new EnumMap<>(PoseId.class);
    private final EnumMap<PathId, PathChain> pathMap = new EnumMap<>(PathId.class);

    public SampleAutoPaths(Follower f, Alliance a) {
        this.f = f;
        this.a = a;
        for (PoseId id : PoseId.values()) {
            poseMap.put(id, id.forAlliance(a));
        }
        for (PathId id : PathId.values()) {
            PathChain chain = id.build(f, this::getPose);
            pathMap.put(id, chain);
        }
    }
    public Pose getPose(PoseId id) {
        return poseMap.get(id);
    }
    public PathChain getPath(PathId id) {
        PathChain chain = pathMap.get(id);
        if (chain == null) {
            throw new IllegalArgumentException("No path built for " + id);
        }
        return chain;
    }
    public Map<PathId, PathChain> getAllPaths() {
        return pathMap;
    }
}
