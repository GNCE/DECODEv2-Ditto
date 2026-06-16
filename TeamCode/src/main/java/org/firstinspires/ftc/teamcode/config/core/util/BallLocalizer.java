package org.firstinspires.ftc.teamcode.config.core.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Artifact;

import java.util.ArrayList;
import java.util.List;

/**
 * Turns the Limelight ball-detector output (see VisionPipeline.py) into tracked balls in
 * field (Pedro) coordinates, and estimates each ball's velocity.
 *
 * <p><b>Why projection happens here and not in Python:</b> the pipeline only knows camera
 * angles. Converting those to a field position needs the robot's odometry pose, and
 * estimating ball velocity needs the robot's own motion removed — both of which live on
 * the robot. By projecting every frame to the field first and then differencing, the
 * robot's translation/rotation cancels out automatically.
 *
 * <p><b>Geometry model:</b> a pinhole camera mounted on the robot, pointing forward along
 * the robot heading (plus an optional yaw offset), tilted down from horizontal by
 * {@link #CAMERA_PITCH_DEG_DOWN}. Each detection's angles define a ray; we intersect that
 * ray with the horizontal plane at the ball's center height ({@link #BALL_RADIUS_IN}) to
 * get the ball's field XY.
 *
 * <p>Field convention matches the rest of the code: X/Y in inches, heading in radians with
 * forward = (cos h, sin h). All camera mount numbers below are placeholders — measure them
 * on the real robot (they are {@code @Configurable} so they can be tuned live in Panels).
 *
 * <h3>Usage</h3>
 * <pre>{@code
 * BallLocalizer localizer = new BallLocalizer();
 * // each loop, after switching the Limelight to the ball pipeline:
 * double[] out = llResult.getPythonOutput();
 * localizer.update(out, follower.getPose(), runtimeNow);
 * List<Ball> balls = localizer.getBalls();   // field coords + velocity
 * }</pre>
 */
@Configurable
public class BallLocalizer {

    // ---- Camera mount geometry (MEASURE THESE on the real robot) ----
    /** Camera lens height above the floor, inches. */
    public static double CAMERA_HEIGHT_IN = 8.56;
    /** Downward tilt of the optical axis from horizontal, degrees. */
    public static double CAMERA_PITCH_DEG_DOWN = 20.0;
    /** Yaw of the camera relative to robot forward, degrees (+ = aimed to robot's right). */
    public static double CAMERA_YAW_OFFSET_DEG = 0.0;
    /** Camera position ahead of the robot's rotation center, inches (+ = forward). */
    public static double CAMERA_FORWARD_OFFSET_IN = 8.43;
    /** Camera position to the left of the robot's rotation center, inches (+ = left). */
    public static double CAMERA_LEFT_OFFSET_IN = 0.0;
    /** Physical ball radius, inches — the ball's center sits this high off the floor. */
    public static double BALL_RADIUS_IN = 2.45;

    // ---- Tracking / velocity ----
    /** A detection within this distance of an existing track (same color) updates it. */
    public static double MATCH_RADIUS_IN = 8.0;
    /** Velocity EMA smoothing in [0,1]; higher = trust the newest sample more. */
    public static double VELOCITY_ALPHA = 0.4;
    /** Position EMA smoothing in [0,1]; higher = snappier, lower = steadier. */
    public static double POSITION_ALPHA = 0.6;
    /** Drop tracks not seen for this long, seconds. */
    public static double STALE_SEC = 0.4;
    /** Ignore implausible jumps that imply faster-than-this ball speed, inches/sec. */
    public static double MAX_BALL_SPEED_IN_S = 120.0;
    /** Reject detections projected outside the field (with this margin), inches. */
    public static double FIELD_SIZE_IN = 141.5;
    public static double FIELD_MARGIN_IN = 6.0;

    private final List<Ball> tracks = new ArrayList<>();
    private int nextId = 0;

    /**
     * Feed one frame of pipeline output. {@code pythonOut} is the raw
     * {@code LLResult.getPythonOutput()} array using the layout documented in
     * VisionPipeline.py. Safe to call with {@code null} / short / all-zero arrays.
     */
    public void update(double[] pythonOut, Pose robotPose, double nowSec) {
        if (pythonOut != null && pythonOut.length >= 1 && robotPose != null) {
            int n = (int) Math.round(pythonOut[0]);
            for (int i = 0; i < n; i++) {
                int base = 1 + 4 * i;
                if (base + 3 >= pythonOut.length) break;

                double angleXDeg = pythonOut[base];
                double angleYDeg = pythonOut[base + 1];
                double radiusPx = pythonOut[base + 2];
                Artifact color = colorFromFlag(pythonOut[base + 3]);
                if (color == Artifact.NONE) continue;

                Pose field = projectToField(angleXDeg, angleYDeg, robotPose);
                if (field == null) continue;
                if (outOfField(field.getX(), field.getY())) continue;

                associate(field, color, radiusPx, nowSec);
            }
        }
        pruneStale(nowSec);
    }

    /** Intersect the detection ray with the plane z = ball-center height; null if it can't hit. */
    private Pose projectToField(double angleXDeg, double angleYDeg, Pose robotPose) {
        double az = Math.toRadians(angleXDeg);  // + = right of camera axis
        double el = Math.toRadians(angleYDeg);  // + = above camera axis
        double h = robotPose.getHeading() - Math.toRadians(CAMERA_YAW_OFFSET_DEG);
        double pitch = Math.toRadians(CAMERA_PITCH_DEG_DOWN);

        // Camera basis expressed in field coordinates (X east, Y north, Z up).
        // Optical axis: along heading, tilted down by pitch.
        double[] forward = {Math.cos(h) * Math.cos(pitch), Math.sin(h) * Math.cos(pitch), -Math.sin(pitch)};
        // Image +x (right of the camera) is horizontal, 90 deg clockwise from heading.
        double[] right = {Math.sin(h), -Math.cos(h), 0.0};
        // Image +y (up) completes the right-handed set: up = forward x right.
        double[] up = cross(forward, right);

        // Ray direction for a detection at (az, el): forward + tan(az)*right + tan(el)*up.
        double tx = Math.tan(az);
        double ty = Math.tan(el);
        double dx = forward[0] + tx * right[0] + ty * up[0];
        double dy = forward[1] + tx * right[1] + ty * up[1];
        double dz = forward[2] + tx * right[2] + ty * up[2];

        // Camera origin in the field.
        double camX = robotPose.getX() + CAMERA_FORWARD_OFFSET_IN * Math.cos(h) - CAMERA_LEFT_OFFSET_IN * Math.sin(h);
        double camY = robotPose.getY() + CAMERA_FORWARD_OFFSET_IN * Math.sin(h) + CAMERA_LEFT_OFFSET_IN * Math.cos(h);
        double camZ = CAMERA_HEIGHT_IN;

        // Solve camZ + t*dz = BALL_RADIUS_IN. Need the ray heading downward to a point below us.
        if (dz >= -1e-6) return null;
        double t = (BALL_RADIUS_IN - camZ) / dz;
        if (t <= 0) return null;

        return new Pose(camX + t * dx, camY + t * dy, 0);
    }

    /** Match to the nearest same-color track within MATCH_RADIUS_IN, else start a new track. */
    private void associate(Pose measured, Artifact color, double radiusPx, double nowSec) {
        Ball best = null;
        double bestDistSq = MATCH_RADIUS_IN * MATCH_RADIUS_IN;
        for (Ball b : tracks) {
            if (b.color != color) continue;
            double d2 = b.position.distSquared(measured);
            if (d2 <= bestDistSq) {
                bestDistSq = d2;
                best = b;
            }
        }

        if (best == null) {
            tracks.add(new Ball(nextId++, color, measured, nowSec, radiusPx));
            return;
        }

        double dt = nowSec - best.lastSeenSec;
        if (dt > 1e-3) {
            // velocity = (measured - last position) / dt
            Pose vRaw = measured.minus(best.position).div(dt);
            // Reject teleporting matches (occlusion swap, bad projection) — keep old velocity.
            if (Math.hypot(vRaw.getX(), vRaw.getY()) <= MAX_BALL_SPEED_IN_S) {
                // EMA: v += alpha * (vRaw - v)
                best.velocity = best.velocity.plus(vRaw.minus(best.velocity).times(VELOCITY_ALPHA));
            }
        }
        best.position = best.position.plus(measured.minus(best.position).times(POSITION_ALPHA));
        best.radiusPx = radiusPx;
        best.lastSeenSec = nowSec;
    }

    private void pruneStale(double nowSec) {
        for (int i = tracks.size() - 1; i >= 0; i--) {
            if (nowSec - tracks.get(i).lastSeenSec > STALE_SEC) tracks.remove(i);
        }
    }

    private boolean outOfField(double x, double y) {
        return x < -FIELD_MARGIN_IN || x > FIELD_SIZE_IN + FIELD_MARGIN_IN
                || y < -FIELD_MARGIN_IN || y > FIELD_SIZE_IN + FIELD_MARGIN_IN;
    }

    private static Artifact colorFromFlag(double flag) {
        int f = (int) Math.round(flag);
        if (f == 1) return Artifact.PURPLE;
        if (f == 2) return Artifact.GREEN;
        return Artifact.NONE;
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[]{
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
        };
    }

    /** Currently tracked balls (live list view; copy if you need to retain it). */
    public List<Ball> getBalls() {
        return new ArrayList<>(tracks);
    }

    public void reset() {
        tracks.clear();
    }
}
