package org.firstinspires.ftc.teamcode.config.core.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Artifact;

import java.util.ArrayList;
import java.util.List;

/**
 * Picks the best three balls to collect and builds a Pedro {@link PathChain} that drives
 * the robot through all three, accounting for each ball's velocity (it aims at where the
 * ball <i>will be</i> when the robot arrives, not where it is now).
 *
 * <p><b>Optimality:</b> with only a handful of candidate balls this is a tiny problem, so
 * we brute-force it. We evaluate every ordered set of up to three balls, simulating the
 * run leg-by-leg: each leg's arrival time advances the clock, and the next ball is
 * predicted forward to that cumulative time before we solve its intercept. The sequence
 * with the smallest total time wins. Brute force (rather than greedy nearest-neighbor)
 * matters because a slightly farther first ball can set up two much shorter later legs.
 *
 * <p>Intercept per leg is solved the same iterative way as the shooter's move-shot lead
 * (see {@link ShotPlanner}): estimate travel time from distance, advance the ball by
 * velocity*time, repeat.
 *
 * <h3>Usage</h3>
 * <pre>{@code
 * BallPathPlanner.Plan plan = planner.plan(follower, follower.getPose(),
 *                                          localizer.getBalls(), Artifact.PURPLE);
 * if (plan != null) follower.followPath(plan.path, true);
 * }</pre>
 */
@Configurable
public class BallPathPlanner {

    /** Assumed average travel speed used to estimate leg times, inches/sec. */
    public static double TRAVEL_SPEED_IN_S = 40.0;
    /** Iterations of the per-leg intercept solver. */
    public static int INTERCEPT_ITERS = 3;
    /** How many balls to string together into one path. */
    public static int MAX_BALLS_IN_PATH = 3;
    /** Don't lead a moving ball more than this far from its current spot, inches. */
    public static double MAX_LEAD_IN = 48.0;

    /** Result of a plan: the path to follow plus the chosen targets and their intercept points. */
    public static class Plan {
        public final PathChain path;
        public final List<Ball> targets;        // chosen balls, in collection order
        public final List<Pose> interceptPoses; // where the robot meets each ball (heading set)
        public final double estTimeSec;

        public Plan(PathChain path, List<Ball> targets, List<Pose> interceptPoses, double estTimeSec) {
            this.path = path;
            this.targets = targets;
            this.interceptPoses = interceptPoses;
            this.estTimeSec = estTimeSec;
        }
    }

    /**
     * Plan a path through up to {@link #MAX_BALLS_IN_PATH} balls.
     *
     * @param follower    used only to obtain a {@link PathBuilder}
     * @param robotPose   current robot pose (start of the run)
     * @param balls       candidate balls (field coords + velocity) from {@link BallLocalizer}
     * @param colorFilter only consider this color; pass {@code null} or {@link Artifact#NONE} for any
     * @return a {@link Plan}, or {@code null} if there are no usable balls
     */
    public Plan plan(Follower follower, Pose robotPose, List<Ball> balls, Artifact colorFilter) {
        List<Ball> candidates = new ArrayList<>();
        for (Ball b : balls) {
            if (colorFilter == null || colorFilter == Artifact.NONE || b.color == colorFilter) {
                candidates.add(b);
            }
        }
        if (candidates.isEmpty()) return null;

        int wanted = Math.min(MAX_BALLS_IN_PATH, candidates.size());

        // Search every ordered selection of `wanted` balls for the cheapest total run.
        Best best = new Best();
        searchSequences(robotPose, candidates, new ArrayList<>(), wanted, best);
        if (best.sequence == null) return null;

        // Re-simulate the winning sequence to recover the intercept poses (with headings).
        List<Pose> intercepts = new ArrayList<>();
        Pose from = robotPose;
        double tCum = 0;
        for (Ball b : best.sequence) {
            Intercept hit = intercept(from, b, tCum);
            double heading = Math.atan2(hit.point.getY() - from.getY(), hit.point.getX() - from.getX());
            Pose hitPose = hit.point.withHeading(heading);
            intercepts.add(hitPose);
            tCum += hit.timeSec;
            from = hitPose;
        }

        // Build the Pedro path: straight legs through the intercept points, the robot's
        // heading swinging to face each ball it is about to drive into.
        PathBuilder builder = follower.pathBuilder();
        Pose prev = robotPose;
        for (Pose hit : intercepts) {
            builder.addPath(new BezierLine(prev, hit))
                    .setLinearHeadingInterpolation(prev.getHeading(), hit.getHeading());
            prev = hit;
        }

        return new Plan(builder.build(), best.sequence, intercepts, best.cost);
    }

    private static class Best {
        List<Ball> sequence = null;
        double cost = Double.POSITIVE_INFINITY;
    }

    /** Recursively try every ordering of `remaining` more balls, tracking the cheapest. */
    private void searchSequences(Pose start, List<Ball> pool, List<Ball> chosen, int remaining, Best best) {
        if (remaining == 0) {
            double cost = sequenceCost(start, chosen);
            if (cost < best.cost) {
                best.cost = cost;
                best.sequence = new ArrayList<>(chosen);
            }
            return;
        }
        for (Ball b : pool) {
            if (chosen.contains(b)) continue;
            chosen.add(b);
            searchSequences(start, pool, chosen, remaining - 1, best);
            chosen.remove(chosen.size() - 1);
        }
    }

    /** Total time to drive the given ordered sequence, advancing balls as time passes. */
    private double sequenceCost(Pose start, List<Ball> sequence) {
        Pose from = start;
        double tCum = 0;
        for (Ball b : sequence) {
            Intercept hit = intercept(from, b, tCum);
            tCum += hit.timeSec;
            from = hit.point;
        }
        return tCum;
    }

    /** Where a leg meets a ball, and how long that leg takes. */
    private static class Intercept {
        final Pose point;
        final double timeSec;

        Intercept(Pose point, double timeSec) {
            this.point = point;
            this.timeSec = timeSec;
        }
    }

    /**
     * Solve where to meet ball {@code b}, starting from {@code from} after {@code tOffset}
     * seconds have already elapsed (so the ball is predicted from its tOffset position).
     */
    private Intercept intercept(Pose from, Ball b, double tOffset) {
        double speed = Math.max(1e-3, TRAVEL_SPEED_IN_S);

        // Ball's position at the moment this leg begins.
        Pose ballStart = b.predict(tOffset);

        Pose target = ballStart;
        double legTime = 0;
        for (int i = 0; i < Math.max(1, INTERCEPT_ITERS); i++) {
            legTime = from.distanceFrom(target) / speed;

            Pose lead = b.velocity.times(legTime);
            double leadMag = Math.hypot(lead.getX(), lead.getY());
            if (leadMag > MAX_LEAD_IN && leadMag > 1e-6) {
                lead = lead.times(MAX_LEAD_IN / leadMag);
            }
            target = ballStart.plus(lead);
        }
        return new Intercept(target, legTime);
    }
}
