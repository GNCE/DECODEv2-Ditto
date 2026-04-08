package org.firstinspires.ftc.teamcode.config.core.util;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;

import java.util.ArrayDeque;

@Configurable
public class ShotPlanner {

    public static class ShotCommand {
        public final Pose predictedRobotPose;
        public final Pose virtualGoal;
        public final double distancePoseUnits;
        public final double targetRpm;
        public final double hoodBaselineDegFromVertical;
        public final double flightTimeSec;
        public final boolean possible;

        public final double robotAxFieldPosePerSec2;
        public final double robotAyFieldPosePerSec2;
        public final double robotAlphaRadPerSec2;
        public ShotCommand(Pose predictedRobotPose,
                           Pose virtualGoal,
                           double distancePoseUnits,
                           double targetRpm,
                           double hoodBaselineDegFromVertical,
                           double flightTimeSec,
                           boolean possible,
                           double robotAxFieldPosePerSec2,
                           double robotAyFieldPosePerSec2,
                           double robotAlphaRadPerSec2) {
            this.predictedRobotPose = predictedRobotPose;
            this.virtualGoal = virtualGoal;
            this.distancePoseUnits = distancePoseUnits;
            this.targetRpm = targetRpm;
            this.hoodBaselineDegFromVertical = hoodBaselineDegFromVertical;
            this.flightTimeSec = flightTimeSec;
            this.possible = possible;
            this.robotAxFieldPosePerSec2 = robotAxFieldPosePerSec2;
            this.robotAyFieldPosePerSec2 = robotAyFieldPosePerSec2;
            this.robotAlphaRadPerSec2 = robotAlphaRadPerSec2;
        }
    }

    private static class MotionSample {
        public final double timeSec;
        public final double vx;
        public final double vy;
        public final double omega;

        public MotionSample(double timeSec, double vx, double vy, double omega) {
            this.timeSec = timeSec;
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
        }
    }

    private static class AccelSample {
        public final double ax;
        public final double ay;
        public final double alpha;

        public AccelSample(double ax, double ay, double alpha) {
            this.ax = ax;
            this.ay = ay;
            this.alpha = alpha;
        }
    }

    public static double POSE_UNITS_TO_METERS = 0.0254;
    public static double EXIT_VEL_M_PER_RPM = 0.00365;

    public static double FUTURE_POSE_PREDICTION_SEC = 0.1;

    public static int ACCEL_REGRESSION_WINDOW_SAMPLES = 10;
    public static double MIN_SAMPLE_SPACING_SEC = 0.008;
    public static double MAX_REGRESSION_HISTORY_SEC = 0.20;
    public static double MAX_LINEAR_ACCEL_POSE_PER_SEC2 = 200.0;
    public static double MAX_ANGULAR_ACCEL_RAD_PER_SEC2 = 20.0;

    public static boolean MOVE_SHOT_ENABLED = true;
    public static int MOVE_SHOT_ITERS = 2;
    public static double MOVE_SHOT_MAX_LEAD_POSE_UNITS = 100.0;

    public static double MAX_RPM = 2800.0;
    public static double MIN_HOOD_ANGLE_DEG = 30;
    public static double MAX_HOOD_ANGLE_DEG = 55;

    public static double FAR_SHOT_THRESHOLD_M = 1.7;
    public static double FAR_SHOT_MIN_HOOD_DEG = 42.0;
    public static double FAR_SHOT_MAX_HOOD_DEG = 46.0;

    public static int RPM_SMOOTHING_WINDOW_SAMPLES = 15;

    public static double GOAL_HEIGHT_M = 1.2575;
    public static double GOAL_LIP_HEIGHT_M = 1.0;
    public static double GOAL_DEPTH_M = 1 * 0.0254;
    public static double LAUNCHER_HEIGHT_ACTUAL_M = 0.38;
    public static double GRAVITY = 9.80665;

    private final double[] distances = {
            50, 53, 57.777, 62.5, 67.5, 96.5, 121.59989031660349, 133.452, 137.83283020070846, 150, 170
    };

    private final double[] velocities = {
            1160, 1240, 1360, 1380, 1460, 1520, 1700, 1840, 1850, 2000, 2400
    };

    private final InterpLUT velocityLut = new InterpLUT();

    private final ElapsedTime motionTimer = new ElapsedTime();
    private final ArrayDeque<MotionSample> motionHistory = new ArrayDeque<>();
    private final ArrayDeque<Double> rpmHistory = new ArrayDeque<>();
    public static JoinedTelemetry tel;

    public ShotPlanner() {
        for (int i = 0; i < distances.length; i++) {
            velocityLut.add(distances[i], velocities[i]);
        }
        velocityLut.createLUT();
        motionTimer.reset();
    }

    private double finiteOrZero(double x) {
        return Double.isFinite(x) ? x : 0.0;
    }

    private double wrapAngleRad(double angleRad) {
        if (!Double.isFinite(angleRad)) return 0.0;
        return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
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

    private double addRpmSample(double rpm) {
        rpm = finiteOrZero(rpm);

        int maxSamples = Math.max(1, RPM_SMOOTHING_WINDOW_SAMPLES);
        rpmHistory.addLast(rpm);
        while (rpmHistory.size() > maxSamples) {
            rpmHistory.pollFirst();
        }

        double sum = 0.0;
        for (double sample : rpmHistory) {
            sum += sample;
        }
        return sum / rpmHistory.size();
    }

    private void addMotionSample(double vx, double vy, double omega) {
        vx = finiteOrZero(vx);
        vy = finiteOrZero(vy);
        omega = finiteOrZero(omega);

        double nowSec = motionTimer.seconds();
        if (!Double.isFinite(nowSec)) return;

        MotionSample last = motionHistory.peekLast();
        if (last != null && (nowSec - last.timeSec) < MIN_SAMPLE_SPACING_SEC) {
            motionHistory.pollLast();
        }

        motionHistory.addLast(new MotionSample(nowSec, vx, vy, omega));

        int maxSamples = Math.max(2, ACCEL_REGRESSION_WINDOW_SAMPLES);
        while (motionHistory.size() > maxSamples) {
            motionHistory.pollFirst();
        }

        while (motionHistory.size() >= 2) {
            MotionSample first = motionHistory.peekFirst();
            MotionSample newest = motionHistory.peekLast();
            if (first == null || newest == null) break;
            if ((newest.timeSec - first.timeSec) <= MAX_REGRESSION_HISTORY_SEC) break;
            motionHistory.pollFirst();
        }
    }

    private double regressionSlope(double[] t, double[] y, int n) {
        if (n < 2) return 0.0;

        double sumT = 0.0;
        double sumY = 0.0;
        for (int i = 0; i < n; i++) {
            if (!Double.isFinite(t[i]) || !Double.isFinite(y[i])) return 0.0;
            sumT += t[i];
            sumY += y[i];
        }

        double meanT = sumT / n;
        double meanY = sumY / n;

        double sxx = 0.0;
        double sxy = 0.0;
        for (int i = 0; i < n; i++) {
            double dt = t[i] - meanT;
            double dy = y[i] - meanY;
            sxx += dt * dt;
            sxy += dt * dy;
        }

        if (!Double.isFinite(sxx) || !Double.isFinite(sxy) || Math.abs(sxx) < 1e-9) return 0.0;
        return sxy / sxx;
    }

    private AccelSample computeAccelerationFromHistory() {
        int n = motionHistory.size();
        if (n < 2) return new AccelSample(0.0, 0.0, 0.0);

        MotionSample first = motionHistory.peekFirst();
        if (first == null) return new AccelSample(0.0, 0.0, 0.0);

        double[] t = new double[n];
        double[] vx = new double[n];
        double[] vy = new double[n];
        double[] omega = new double[n];

        int i = 0;
        for (MotionSample sample : motionHistory) {
            t[i] = sample.timeSec - first.timeSec;
            vx[i] = sample.vx;
            vy[i] = sample.vy;
            omega[i] = sample.omega;
            i++;
        }

        double ax = regressionSlope(t, vx, n);
        double ay = regressionSlope(t, vy, n);
        double alpha = regressionSlope(t, omega, n);

        ax = MathUtils.clamp(finiteOrZero(ax), -Math.abs(MAX_LINEAR_ACCEL_POSE_PER_SEC2), Math.abs(MAX_LINEAR_ACCEL_POSE_PER_SEC2));
        ay = MathUtils.clamp(finiteOrZero(ay), -Math.abs(MAX_LINEAR_ACCEL_POSE_PER_SEC2), Math.abs(MAX_LINEAR_ACCEL_POSE_PER_SEC2));
        alpha = MathUtils.clamp(finiteOrZero(alpha), -Math.abs(MAX_ANGULAR_ACCEL_RAD_PER_SEC2), Math.abs(MAX_ANGULAR_ACCEL_RAD_PER_SEC2));

        return new AccelSample(ax, ay, alpha);
    }

    public void resetMotionHistory() {
        motionHistory.clear();
        motionTimer.reset();
    }

    private Pose predictFutureRobotPose(Pose robotPose,
                                        double robotVxFieldPosePerSec,
                                        double robotVyFieldPosePerSec,
                                        double robotOmegaRadPerSec,
                                        double robotAxFieldPosePerSec2,
                                        double robotAyFieldPosePerSec2,
                                        double robotAlphaRadPerSec2) {
        double t = Math.max(0.0, finiteOrZero(FUTURE_POSE_PREDICTION_SEC));
        double t2 = t * t;

        double predictedX = robotPose.getX()
                + robotVxFieldPosePerSec * t
                + 0.5 * robotAxFieldPosePerSec2 * t2;

        double predictedY = robotPose.getY()
                + robotVyFieldPosePerSec * t
                + 0.5 * robotAyFieldPosePerSec2 * t2;

        double predictedHeading = wrapAngleRad(
                robotPose.getHeading()
                        + robotOmegaRadPerSec * t
                        + 0.5 * robotAlphaRadPerSec2 * t2
        );

        return new Pose(predictedX, predictedY, predictedHeading);
    }

    private static boolean isValidArc(double theta, double xLip, double v2, double g,
                                      double minHoodDeg, double maxHoodDeg) {
        if (Double.isNaN(theta)) return false;

        double hoodDeg = 90.0 - Math.toDegrees(theta);
        if (hoodDeg < minHoodDeg || hoodDeg > maxHoodDeg) return false;

        if (xLip <= 0.0) return true;

        double cos = Math.cos(theta);
        if (Math.abs(cos) < 1e-6) return false;

        double hAtLip = xLip * Math.tan(theta)
                - (g * xLip * xLip) / (2.0 * v2 * cos * cos);

        return (hAtLip + LAUNCHER_HEIGHT_ACTUAL_M) >= GOAL_LIP_HEIGHT_M;
    }

    public static double solveHoodDeg(double distPoseUnits, double rpm) {
        double x = distPoseUnits * POSE_UNITS_TO_METERS;
        double v = Math.abs(rpm) * EXIT_VEL_M_PER_RPM;
        double dhBack = GOAL_HEIGHT_M - LAUNCHER_HEIGHT_ACTUAL_M;
        double xLip = x - GOAL_DEPTH_M;

        if (x <= 1e-6 || v <= 1e-6) return Double.NaN;

        double g = GRAVITY;
        double v2 = v * v;
        double v4 = v2 * v2;

        boolean isFarShot = x > FAR_SHOT_THRESHOLD_M;
        double minHood = isFarShot ? FAR_SHOT_MIN_HOOD_DEG : MIN_HOOD_ANGLE_DEG;
        double maxHood = isFarShot ? FAR_SHOT_MAX_HOOD_DEG : MAX_HOOD_ANGLE_DEG;

        double thetaLo = Double.NaN;
        double thetaHi = Double.NaN;

        double discBack = v4 - g * (g * x * x + 2.0 * dhBack * v2);
        if (discBack >= 0.0) {
            double sqrtBack = Math.sqrt(discBack);
            thetaLo = Math.atan((v2 - sqrtBack) / (g * x));
            thetaHi = Math.atan((v2 + sqrtBack) / (g * x));
        }

        if (isValidArc(thetaLo, xLip, v2, g, minHood, maxHood)) {
            tel.addLine("[AUTO AIM] LOW ARC");
            return 90.0 - Math.toDegrees(thetaLo);
        }

        if (isValidArc(thetaHi, xLip, v2, g, minHood, maxHood)) {
            tel.addLine("[AUTO AIM] HIGH ARC");
            return 90.0 - Math.toDegrees(thetaHi);
        }

        tel.addLine("[AUTO AIM] IMPOSSIBLE");
        return Double.NaN;
    }

    private double estimateFlightTimeSec(double distPoseUnits, double rpm, double hoodDegFromVertical) {
        double xMeters = distPoseUnits * POSE_UNITS_TO_METERS;
        double theta = hoodDegToThetaRadFromHorizontal(hoodDegFromVertical);

        double vExit = Math.abs(rpm) * EXIT_VEL_M_PER_RPM;
        double vHoriz = vExit * Math.cos(theta);

        if (vHoriz < 1e-3) return Double.NaN;
        return xMeters / vHoriz;
    }

    private Pose computeVirtualGoalIter(Pose robotPoseForPlanning,
                                        Pose realGoalPose,
                                        double launchVxFieldPosePerSec,
                                        double launchVyFieldPosePerSec,
                                        double rpm) {
        if (!MOVE_SHOT_ENABLED) return realGoalPose;

        Pose virtualGoal = realGoalPose;

        for (int i = 0; i < MOVE_SHOT_ITERS; i++) {
            double dx = virtualGoal.getX() - robotPoseForPlanning.getX();
            double dy = virtualGoal.getY() - robotPoseForPlanning.getY();
            double distPose = Math.hypot(dx, dy);
            double distClamped = clampDist(distPose);

            double hood = solveHoodDeg(distClamped, rpm);
            if (!Double.isFinite(hood)) return virtualGoal;

            double t = estimateFlightTimeSec(distClamped, rpm, hood);
            if (!Double.isFinite(t)) return virtualGoal;

            double leadX = launchVxFieldPosePerSec * t;
            double leadY = launchVyFieldPosePerSec * t;

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

    public ShotCommand plan(Pose robotPose,
                            double robotVxFieldPosePerSec,
                            double robotVyFieldPosePerSec,
                            double robotOmegaRadPerSec,
                            Pose realGoalPose, double rpm) {

        double smoothedRpm = addRpmSample(rpm);

        addMotionSample(robotVxFieldPosePerSec, robotVyFieldPosePerSec, robotOmegaRadPerSec);
        AccelSample accel = computeAccelerationFromHistory();

        Pose predictedRobotPose = predictFutureRobotPose(
                robotPose,
                finiteOrZero(robotVxFieldPosePerSec),
                finiteOrZero(robotVyFieldPosePerSec),
                finiteOrZero(robotOmegaRadPerSec),
                accel.ax,
                accel.ay,
                accel.alpha
        );

        double dt = Math.max(0.0, finiteOrZero(FUTURE_POSE_PREDICTION_SEC));
        double predictedLaunchVx = finiteOrZero(robotVxFieldPosePerSec) + accel.ax * dt;
        double predictedLaunchVy = finiteOrZero(robotVyFieldPosePerSec) + accel.ay * dt;

        Pose virtualGoal = computeVirtualGoalIter(
                predictedRobotPose,
                realGoalPose,
                predictedLaunchVx,
                predictedLaunchVy,
                smoothedRpm
        );

        double dx = virtualGoal.getX() - predictedRobotPose.getX();
        double dy = virtualGoal.getY() - predictedRobotPose.getY();
        double distPose = Math.hypot(dx, dy);
        double distClamped = clampDist(distPose);

        double targetRPM = Range.clip(velocityLut.get(distClamped), 0.0, MAX_RPM);
        double hood = solveHoodDeg(distClamped, smoothedRpm);
        boolean possible = Double.isFinite(hood);

        double flightTimeSec = possible ? estimateFlightTimeSec(distClamped, smoothedRpm, hood) : Double.NaN;

        return new ShotCommand(
                predictedRobotPose,
                virtualGoal,
                distClamped,
                targetRPM,
                hood,
                flightTimeSec,
                possible,
                accel.ax,
                accel.ay,
                accel.alpha
        );
    }

    public double getLutRpm(double distPoseUnits) {
        return Range.clip(velocityLut.get(clampDist(distPoseUnits)), 0.0, MAX_RPM);
    }

    public double getPhysicsHoodDeg(double distPoseUnits) {
        return solveHoodDeg(clampDist(distPoseUnits), getLutRpm(distPoseUnits));
    }
}