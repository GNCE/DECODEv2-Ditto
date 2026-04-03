package org.firstinspires.ftc.teamcode.config.core.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.pedropathing.geometry.Pose;

/**
 * Pure-math planner:
 *  - Uses LUT(distance)->RPM
 *  - Computes hood angle from physics (projectile lo-branch solution)
 *  - Computes virtual goal for shooting on the move (iterative lead)
 *  - Provides a single coherent "ShotCommand" to feed Turret + Shooter
 *
 * Pose units assumed inches by default (POSE_UNITS_TO_METERS = 0.0254).
 * Hood angle convention: degrees FROM VERTICAL (0 up, 90 horizontal).
 */
@Configurable
public class ShotPlanner {

    public static class ShotCommand {
        public final Pose virtualGoal;
        public final double distancePoseUnits;
        public final double targetRpm;
        public final double hoodBaselineDegFromVertical;
        public final double flightTimeSec;

        public ShotCommand(Pose virtualGoal,
                           double distancePoseUnits,
                           double targetRpm,
                           double hoodBaselineDegFromVertical,
                           double flightTimeSec) {
            this.virtualGoal = virtualGoal;
            this.distancePoseUnits = distancePoseUnits;
            this.targetRpm = targetRpm;
            this.hoodBaselineDegFromVertical = hoodBaselineDegFromVertical;
            this.flightTimeSec = flightTimeSec;
        }
    }

    // -------------------- Shared geometry / conversion --------------------
    // Pedro Pose units are inches (typical). If meters, set to 1.0.
    public static double POSE_UNITS_TO_METERS = 0.0254;

    // Calibrated from your LUT + heights earlier:
    // v_exit (m/s) ~= EXIT_VEL_M_PER_RPM * RPM
    public static double EXIT_VEL_M_PER_RPM = 0.0039;

    // Lead tuning
    public static boolean MOVE_SHOT_ENABLED = true;
    public static int MOVE_SHOT_ITERS = 2;
    public static double MOVE_SHOT_MAX_LEAD_POSE_UNITS = 24.0;

    // Clamp output setpoints (match your Shooter constraints)
    public static double MAX_RPM = 2800.0;

    // Hood clamp (keep in sync with Shooter)
    public static double MIN_HOOD_ANGLE_DEG = 30;
    public static double MAX_HOOD_ANGLE_DEG = 55;

    // -------------------- Physics constants (for hood angle solver) --------------------
    public static double GOAL_HEIGHT_M = 1.14;
    public static double LAUNCHER_HEIGHT_ACTUAL_M = 0.38;
    public static double GRAVITY = 9.80665;

    // -------------------- LUT data (from your latest Shooter) --------------------
    private final double[] distances = {
            //120.99649140247442, 130.998778563601, 140.51913130739305
            50, 53, 57.777, 62.5, 67.5, 96.5, 121.59989031660349, 133.452, 137.83283020070846, 150, 170
    };
    private final double[] velocities = {
            // 1900, 2000, 2100
            1160, 1240, 1360, 1380, 1460, 1520, 1700, 1840, 1800, 2000, 2400
    };
    // Hood angle LUT commented out — replaced by physics solver
    // private final double[] hoodAngles = {
    //         // 49, 49, 50
    //         30, 35, 38.5, 39, 41, 41, 43, 44, 44, 44, 47
    // };

    private final InterpLUT velocityLut = new InterpLUT();
    // Hood angle LUT commented out — replaced by physics solver
    // private final InterpLUT hoodAngleLut = new InterpLUT();

    public ShotPlanner() {
        for (int i = 0; i < distances.length; i++) {
            velocityLut.add(distances[i], velocities[i]);
            // hoodAngleLut.add(distances[i], hoodAngles[i]); // commented out — using physics now
        }
        velocityLut.createLUT();
        // hoodAngleLut.createLUT(); // commented out — using physics now
    }

    private double clampDist(double distPoseUnits) {
        return MathUtils.clamp(
                distPoseUnits,
                distances[0] * 1.01,
                distances[distances.length - 1] * 0.99
        );
    }

    private double hoodDegToThetaRadFromHorizontal(double hoodDegFromVertical) {
        return Math.toRadians(90.0 - hoodDegFromVertical);
    }

    // -------------------- Physics hood solver --------------------
    // Solves projectile equations for the lo-branch launch angle (flatter trajectory)
    // given horizontal distance (m), exit velocity (m/s), and height difference (m).
    // Returns hood angle in degrees from vertical, or fallback to MIN if no solution.
    private double solveHoodDeg(double distPoseUnits, double rpm) {
        double x = distPoseUnits * POSE_UNITS_TO_METERS;
        double v = Math.abs(rpm) * EXIT_VEL_M_PER_RPM;
        double dh = GOAL_HEIGHT_M - LAUNCHER_HEIGHT_ACTUAL_M;

        if (x <= 1e-6 || v <= 1e-6) return MIN_HOOD_ANGLE_DEG;

        double g = GRAVITY;
        double v2 = v * v;
        double v4 = v2 * v2;

        double disc = v4 - g * (g * x * x + 2.0 * dh * v2);
        if (disc < 0.0) return MIN_HOOD_ANGLE_DEG; // no solution, fallback

        double sqrt = Math.sqrt(disc);
        double tanLo = (v2 - sqrt) / (g * x); // lo branch = flatter trajectory
        double thetaLo = Math.atan(tanLo);     // radians from horizontal

        double hoodDeg = 90.0 - Math.toDegrees(thetaLo); // convert to deg from vertical
        return Range.clip(hoodDeg, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
    }

    private double estimateFlightTimeSec(double distPoseUnits, double rpm, double hoodDegFromVertical) {
        double xMeters = distPoseUnits * POSE_UNITS_TO_METERS;
        double theta = hoodDegToThetaRadFromHorizontal(hoodDegFromVertical);

        double vExit = Math.abs(rpm) * EXIT_VEL_M_PER_RPM; // m/s
        double vHoriz = vExit * Math.cos(theta);           // m/s

        if (vHoriz < 1e-3) return 0.0;
        return xMeters / vHoriz;
    }

    private Pose computeVirtualGoalIter(Pose robotPose, Pose realGoalPose,
                                        double robotVxFieldPosePerSec, double robotVyFieldPosePerSec) {
        if (!MOVE_SHOT_ENABLED) return realGoalPose;

        Pose virtualGoal = realGoalPose;

        for (int i = 0; i < MOVE_SHOT_ITERS; i++) {
            double dx = virtualGoal.getX() - robotPose.getX();
            double dy = virtualGoal.getY() - robotPose.getY();
            double distPose = Math.hypot(dx, dy);

            double distClamped = clampDist(distPose);

            double rpm = Range.clip(velocityLut.get(distClamped), 0.0, MAX_RPM);
            double hood = solveHoodDeg(distClamped, rpm); // physics solver replaces hoodAngleLut.get()

            double t = estimateFlightTimeSec(distClamped, rpm, hood);

            double leadX = robotVxFieldPosePerSec * t;
            double leadY = robotVyFieldPosePerSec * t;

            double leadMag = Math.hypot(leadX, leadY);
            if (leadMag > MOVE_SHOT_MAX_LEAD_POSE_UNITS && leadMag > 1e-6) {
                double s = MOVE_SHOT_MAX_LEAD_POSE_UNITS / leadMag;
                leadX *= s;
                leadY *= s;
            }

            virtualGoal = new Pose(
                    realGoalPose.getX() - leadX,
                    realGoalPose.getY() - leadY,
                    realGoalPose.getHeading()
            );
        }

        return virtualGoal;
    }

    /**
     * Plan a shot from robot state.
     *
     * @param robotPose Field pose (Pose units, typically inches)
     * @param robotVxFieldPosePerSec Field-relative vx (Pose units / sec)
     * @param robotVyFieldPosePerSec Field-relative vy (Pose units / sec)
     * @param realGoalPose Real goal field pose
     */
    public ShotCommand plan(Pose robotPose,
                            double robotVxFieldPosePerSec,
                            double robotVyFieldPosePerSec,
                            Pose realGoalPose) {

        Pose virtualGoal = computeVirtualGoalIter(robotPose, realGoalPose, robotVxFieldPosePerSec, robotVyFieldPosePerSec);

        double dx = virtualGoal.getX() - robotPose.getX();
        double dy = virtualGoal.getY() - robotPose.getY();
        double distPose = Math.hypot(dx, dy);

        double distClamped = clampDist(distPose);

        double rpm = Range.clip(velocityLut.get(distClamped), 0.0, MAX_RPM);
        double hood = solveHoodDeg(distClamped, rpm); // physics solver replaces hoodAngleLut.get()

        double t = estimateFlightTimeSec(distClamped, rpm, hood);

        return new ShotCommand(virtualGoal, distClamped, rpm, hood, t);
    }

    // Optional: expose raw LUT outputs (useful for debugging)
    public double getLutRpm(double distPoseUnits) {
        return Range.clip(velocityLut.get(clampDist(distPoseUnits)), 0.0, MAX_RPM);
    }

    // Hood angle LUT helper commented out — replaced by physics solver
    // public double getLutHoodDeg(double distPoseUnits) {
    //     return Range.clip(hoodAngleLut.get(clampDist(distPoseUnits)), MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
    // }

    // Optional: expose physics hood angle directly (useful for debugging)
    public double getPhysicsHoodDeg(double distPoseUnits) {
        return solveHoodDeg(clampDist(distPoseUnits), getLutRpm(distPoseUnits));
    }
}