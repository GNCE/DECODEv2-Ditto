package org.firstinspires.ftc.teamcode.config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.util.MathUtils;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Configurable
public class Shooter extends SubsysCore {
    private final MotorGroup flywheel;
    private final Motor.Encoder encoder;
    private final Servo hood;

    public static double INACTIVE_VELOCITY = 0.0;
    public static double VELOCITY_READY_THRESHOLD = 100.0;

    public static double kp = 0.0006;
    public static double ki = 0.0;
    public static double kd = 0;
    public static double kV = 0.000400;

    private double currentTargetVelocity = 0.0;
    private double currentTargetHoodAngle = 30;

    public static double LAUNCHER_HEIGHT_M = 0.26839625; // legacy (not used by physics below)

    // Hood angle range (deg FROM VERTICAL: 0° up, 90° horizontal)
    public static double MIN_HOOD_ANGLE_DEG = 29;
    public static double MAX_HOOD_ANGLE_DEG = 52.514;

    // Servo positions at angle limits
    public static double HOOD_MAX_SERVO_POS = 0.9;
    public static double HOOD_GEAR_RATIO = (double) 300 / 44;
    public static double HOOD_SERVO_RANGE = 355.0;
    public static double IDLE_HOOD_ANGLE_DEG = MAX_HOOD_ANGLE_DEG;

    // Kept (harmless) even though planner owns LUT now
    private final double[] distances = {
            24.508, 39.477, 65.4485, 87.6797, 112.16, 118.97, 128.7672 ,155.791
    };
    private final double[] velocities = {
            1200, 1300, 1340, 1540, 1700, 1740, 1800, 2000
    };
    private final double[] hoodAngles = {
            29, 38, 45, 50, 52, 52.5, 52.5, 52.5
    };

    PIDFController pidfController;
    private final InterpLUT velocityLut = new InterpLUT();
    private final InterpLUT hoodAngleLut = new InterpLUT();

    private double currentDistanceM = 0.0;
    boolean active = false;

    // Keep semantics: when true, Shooter should NOT run normal operation.
    public static boolean testModeOnly = false;

    // ---------------- Planner inputs (set by ShotPlanner caller) ----------------
    private double plannedDistPoseUnits = 0.0;        // distance to VIRTUAL goal (inches)
    private double plannedTargetRpm = 0.0;            // rpm from ShotPlanner LUT
    private double plannedHoodBaselineDeg = IDLE_HOOD_ANGLE_DEG; // baseline hood from ShotPlanner LUT

    public void setPlannedShot(double distPoseUnits, double targetRpm, double hoodBaselineDegFromVertical) {
        plannedDistPoseUnits = distPoseUnits;
        plannedTargetRpm = targetRpm;
        plannedHoodBaselineDeg = hoodBaselineDegFromVertical;
    }
    // --------------------------------------------------------------------------

    public static double NOMINAL_VOLTAGE = 13.5;

    // ---------------- PHYSICS-BASED VELOCITY COMPENSATION ----------------
    public static double POSE_UNITS_TO_METERS = 0.0254;

    public static double GOAL_HEIGHT_M = 1.14;
    public static double LAUNCHER_HEIGHT_ACTUAL_M = 0.38;

    public static double GRAVITY = 9.80665;

    public static double EXIT_VEL_M_PER_RPM = 0.0039;

    public static double PHYS_COMP_MAX_DELTA_DEG = 6.0;
    public static double PHYS_COMP_MIN_FRAC = 0.55;
    public static boolean PHYS_COMP_ENABLED = true;

    private double hoodToThetaRad(double hoodDegFromVertical) {
        return Math.toRadians(90.0 - hoodDegFromVertical);
    }

    private double thetaToHoodDeg(double thetaRadFromHorizontal) {
        return 90.0 - Math.toDegrees(thetaRadFromHorizontal);
    }

    private double solveThetaRad(double x, double v, double dh, double thetaHintRad) {
        if (x <= 1e-6 || v <= 1e-6) return Double.NaN;

        double g = GRAVITY;
        double v2 = v * v;
        double v4 = v2 * v2;

        double disc = v4 - g * (g * x * x + 2.0 * dh * v2);
        if (disc < 0.0) return Double.NaN;

        double sqrt = Math.sqrt(disc);

        double tanHi = (v2 + sqrt) / (g * x);
        double tanLo = (v2 - sqrt) / (g * x);

        double thHi = Math.atan(tanHi);
        double thLo = Math.atan(tanLo);

        double dHi = Math.abs(thHi - thetaHintRad);
        double dLo = Math.abs(thLo - thetaHintRad);
        return (dHi <= dLo) ? thHi : thLo;
    }

    private double physicsCompensatedHoodDeg(double distPoseUnits,
                                             double targetRPM,
                                             double actualRPM,
                                             double hoodBaselineDegFromVertical) {

        if (!PHYS_COMP_ENABLED) {
            return Range.clip(hoodBaselineDegFromVertical, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        }

        double tgt = Math.abs(targetRPM);
        double act = Math.abs(actualRPM);

        if (tgt < 1.0) {
            return Range.clip(hoodBaselineDegFromVertical, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        }
        if (act < PHYS_COMP_MIN_FRAC * tgt) {
            return Range.clip(hoodBaselineDegFromVertical, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        }

        double x = distPoseUnits * POSE_UNITS_TO_METERS;
        double dh = GOAL_HEIGHT_M - LAUNCHER_HEIGHT_ACTUAL_M;

        double vTarget = tgt * EXIT_VEL_M_PER_RPM;
        double vActual = act * EXIT_VEL_M_PER_RPM;

        double thetaHint = hoodToThetaRad(hoodBaselineDegFromVertical);

        double thetaTarget = solveThetaRad(x, vTarget, dh, thetaHint);
        double thetaActual = solveThetaRad(x, vActual, dh, thetaHint);

        if (!Double.isFinite(thetaTarget) || !Double.isFinite(thetaActual)) {
            return Range.clip(hoodBaselineDegFromVertical, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        }

        double dTheta = thetaActual - thetaTarget;
        double thetaCmd = thetaHint + dTheta;

        double hoodCmd = thetaToHoodDeg(thetaCmd);

        double deltaHood = hoodCmd - hoodBaselineDegFromVertical;
        deltaHood = Range.clip(deltaHood, -PHYS_COMP_MAX_DELTA_DEG, PHYS_COMP_MAX_DELTA_DEG);

        hoodCmd = hoodBaselineDegFromVertical + deltaHood;
        return Range.clip(hoodCmd, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
    }
    // --------------------------------------------------------------------

    public Shooter() {
        Motor m1 = new MotorEx(h, "shooter1", Motor.GoBILDA.BARE), m2 = new MotorEx(h, "shooter2", Motor.GoBILDA.BARE);
        m1.setInverted(true);
        flywheel = new MotorGroup(m1, m2);
        flywheel.setRunMode(Motor.RunMode.RawPower);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        encoder = m1.encoder;
        encoder.setDirection(Motor.Direction.FORWARD);

        hood = h.get(Servo.class, "hood");

        for (int i = 0; i < distances.length; i++) {
            velocityLut.add(distances[i], velocities[i]);
            hoodAngleLut.add(distances[i], hoodAngles[i]);
        }
        velocityLut.createLUT();
        hoodAngleLut.createLUT();

        pidfController = new PIDFController(kp, ki, kd, kV);

        testModeOnly = false;
        setDefaultCommand(new RunCommand(() -> active = true, this));
    }

    public void setActive(boolean active) {
        this.active = active;
    }

    public boolean isActive() {
        return active;
    }

    public double getTargetVelocity() {
        return currentTargetVelocity;
    }

    public boolean readyToShoot() {
        if (!active) return false;
        return Math.abs(getTargetVelocity() - getVelocity()) < VELOCITY_READY_THRESHOLD;
    }

    public void input(Pose robotPose, Pose goalPose) {
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        currentDistanceM = Math.hypot(dx, dy);
    }

    private double hoodAngleToServoPos(double hoodAngleDeg) {
        double clipped = Range.clip(hoodAngleDeg, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        double t = (MAX_HOOD_ANGLE_DEG - clipped);
        return HOOD_MAX_SERVO_POS - t * HOOD_GEAR_RATIO / HOOD_SERVO_RANGE;
    }

    public double getVelocity() {
        return -encoder.getCorrectedVelocity();
    }

    @Override
    public void periodic() {
        pidfController.setPIDF(kp, ki, kd, kV);

        // Rule you asked for:
        // - if testModeOnly is OFF => planner mode (use planned targets)
        // - if testModeOnly is ON  => do NOT run normal operation (preserve original semantics)
        if (active && !testModeOnly) {
            currentTargetVelocity = Range.clip(plannedTargetRpm, 0.0, 2800);
            currentTargetHoodAngle = Range.clip(plannedHoodBaselineDeg, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        } else {
            currentTargetVelocity = INACTIVE_VELOCITY;
            currentTargetHoodAngle = IDLE_HOOD_ANGLE_DEG;
        }

        double targetPower = MathUtils.clamp(pidfController.calculate(getVelocity(), getTargetVelocity()), -1, 1);
        flywheel.set(targetPower);

        double hoodCmdDeg = (active && !testModeOnly)
                ? physicsCompensatedHoodDeg(plannedDistPoseUnits, currentTargetVelocity, getVelocity(), currentTargetHoodAngle)
                : currentTargetHoodAngle;

        hood.setPosition(hoodAngleToServoPos(hoodCmdDeg));

        t.addData("Shooter Velocity Error", getTargetVelocity() - getVelocity());
        t.addData("Shooter Target Power", targetPower);
        t.addData("Shooter Distance (M)", currentDistanceM);
        t.addData("Shooter Velocity", getVelocity());
        t.addData("Shooter Target Velocity", getTargetVelocity());
        t.addData("Shooter Ready", readyToShoot());
        t.addData("Shooter testModeOnly", testModeOnly);
        t.addData("Shooter Planned Dist (in)", plannedDistPoseUnits);
        t.addData("Shooter Hood Baseline (deg)", currentTargetHoodAngle);
        t.addData("Shooter Hood Cmd (deg)", hoodCmdDeg);
    }
}
