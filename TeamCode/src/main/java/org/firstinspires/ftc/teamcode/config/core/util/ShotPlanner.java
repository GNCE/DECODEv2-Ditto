package org.firstinspires.ftc.teamcode.config.core.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.pedropathing.geometry.Pose;

/**
 * Pure-math planner:
 *  - Uses LUT(distance)->(RPM, hoodAngle)
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
    public static double MOVE_SHOT_MAX_LEAD_POSE_UNITS = 18.0;

    // Clamp output setpoints (match your Shooter constraints)
    public static double MAX_RPM = 2800.0;

    // Hood clamp (keep in sync with Shooter)
    public static double MIN_HOOD_ANGLE_DEG = 29.0;
    public static double MAX_HOOD_ANGLE_DEG = 52.514;

    // -------------------- LUT data (from your latest Shooter) --------------------
    private final double[] distances = {
            24.508, 39.477, 65.4485, 87.6797, 112.16, 118.97, 128.7672 ,155.791
    };
    private final double[] velocities = {
            1200, 1300, 1340, 1540, 1700, 1740, 1800, 2000
    };
    private final double[] hoodAngles = {
            29, 38, 45, 50, 52, 52.5, 52.5, 52.5
    };

    private final InterpLUT velocityLut = new InterpLUT();
    private final InterpLUT hoodAngleLut = new InterpLUT();

    public ShotPlanner() {
        for (int i = 0; i < distances.length; i++) {
            velocityLut.add(distances[i], velocities[i]);
            hoodAngleLut.add(distances[i], hoodAngles[i]);
        }
        velocityLut.createLUT();
        hoodAngleLut.createLUT();
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
            double hood = Range.clip(hoodAngleLut.get(distClamped), MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);

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
        double hood = Range.clip(hoodAngleLut.get(distClamped), MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);

        double t = estimateFlightTimeSec(distClamped, rpm, hood);

        return new ShotCommand(virtualGoal, distClamped, rpm, hood, t);
    }

    // Optional: expose raw LUT outputs (useful for debugging)
    public double getLutRpm(double distPoseUnits) {
        return Range.clip(velocityLut.get(clampDist(distPoseUnits)), 0.0, MAX_RPM);
    }

    public double getLutHoodDeg(double distPoseUnits) {
        return Range.clip(hoodAngleLut.get(clampDist(distPoseUnits)), MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
    }
}
