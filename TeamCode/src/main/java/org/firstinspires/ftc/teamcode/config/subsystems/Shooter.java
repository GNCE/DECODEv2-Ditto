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

    public static double LAUNCHER_HEIGHT_M = 0.38; // ball exit height

    // Hood angle range (deg FROM VERTICAL: 0° up, 90° horizontal)
    public static double MIN_HOOD_ANGLE_DEG = 29; // was 28
    public static double MAX_HOOD_ANGLE_DEG = 52.514; // 28 - 52.514

    // Servo positions at angle limits
    public static double HOOD_MAX_SERVO_POS = 0.9; // higher pos = lower (per your mapping)
    public static double HOOD_GEAR_RATIO =  (double)300/44;
    public static double HOOD_SERVO_RANGE = 355.0;
    public static double IDLE_HOOD_ANGLE_DEG = MAX_HOOD_ANGLE_DEG;

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
    public static boolean testModeOnly = false;

    public static double NOMINAL_VOLTAGE = 13.5;

    // ---------------- PHYSICS-BASED VELOCITY COMPENSATION ----------------
    // If Pedro Pose units are inches (typical), keep 0.0254. If meters, set 1.0.
    public static double POSE_UNITS_TO_METERS = 0.0254;

    // Height of the target point you want the ball to pass through (meters).
    public static double GOAL_HEIGHT_M = 1.14;

    public static double GRAVITY = 9.80665;

    // Exit speed model: v_exit(m/s) = EXIT_VEL_M_PER_RPM * flywheelRPM
    public static double EXIT_VEL_M_PER_RPM = 0.00391;

    // Clamp how much physics can change the LUT (deg FROM VERTICAL)
    public static double PHYS_COMP_MAX_DELTA_DEG = 6.0;

    // Don’t apply physics when very far from target speed (avoids huge early-spinup moves)
    public static double PHYS_COMP_MIN_FRAC = 0.55;

    // Toggle if you want to disable physics quickly
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
                                             double hoodLutDegFromVertical) {

        if (!PHYS_COMP_ENABLED) {
            return Range.clip(hoodLutDegFromVertical, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        }

        double tgt = Math.abs(targetRPM);
        double act = Math.abs(actualRPM);

        if (tgt < 1.0) {
            return Range.clip(hoodLutDegFromVertical, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        }
        if (act < PHYS_COMP_MIN_FRAC * tgt) {
            return Range.clip(hoodLutDegFromVertical, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        }

        double x = distPoseUnits * POSE_UNITS_TO_METERS;
        double dh = GOAL_HEIGHT_M - LAUNCHER_HEIGHT_M;

        double vTarget = tgt * EXIT_VEL_M_PER_RPM;
        double vActual = act * EXIT_VEL_M_PER_RPM;

        double thetaHint = hoodToThetaRad(hoodLutDegFromVertical);

        double thetaTarget = solveThetaRad(x, vTarget, dh, thetaHint);
        double thetaActual = solveThetaRad(x, vActual, dh, thetaHint);

        if (!Double.isFinite(thetaTarget) || !Double.isFinite(thetaActual)) {
            return Range.clip(hoodLutDegFromVertical, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        }

        double dTheta = thetaActual - thetaTarget;
        double thetaCmd = thetaHint + dTheta;

        double hoodCmd = thetaToHoodDeg(thetaCmd);

        double deltaHood = hoodCmd - hoodLutDegFromVertical;
        deltaHood = Range.clip(deltaHood, -PHYS_COMP_MAX_DELTA_DEG, PHYS_COMP_MAX_DELTA_DEG);

        hoodCmd = hoodLutDegFromVertical + deltaHood;
        return Range.clip(hoodCmd, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
    }
    // --------------------------------------------------------------------

    public Shooter() {
        Motor m1 = new MotorEx(h, "shooter1", Motor.GoBILDA.BARE),
                m2 = new MotorEx(h, "shooter2", Motor.GoBILDA.BARE);

        m1.setInverted(true);
        flywheel = new MotorGroup(m1, m2);
        flywheel.setRunMode(Motor.RunMode.RawPower);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        encoder = m1.encoder;
        encoder.setDirection(Motor.Direction.FORWARD); // apparently does not do anything

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

        double dist = MathUtils.clamp(
                currentDistanceM,
                distances[0] * 1.01,
                distances[distances.length - 1] * 0.99
        );

        if (active && !testModeOnly) {
            double lutVelocity = velocityLut.get(dist);
            double lutHoodAngle = hoodAngleLut.get(dist);
            currentTargetVelocity = Range.clip(lutVelocity, 0.0, 2800);
            currentTargetHoodAngle = Range.clip(lutHoodAngle, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        } else {
            currentTargetVelocity = INACTIVE_VELOCITY;
            currentTargetHoodAngle = IDLE_HOOD_ANGLE_DEG;
        }

        double targetPower = MathUtils.clamp(
                pidfController.calculate(getVelocity(), getTargetVelocity()),
                -1, 1
        );
        flywheel.set(targetPower);

        double hoodCmdDeg = (active && !testModeOnly)
                ? physicsCompensatedHoodDeg(dist, getTargetVelocity(), getVelocity(), currentTargetHoodAngle)
                : currentTargetHoodAngle;

        double servoPos = hoodAngleToServoPos(hoodCmdDeg);
        hood.setPosition(servoPos);

        t.addData("Shooter Velocity Error", getTargetVelocity() - getVelocity());
        t.addData("Shooter Target Power", targetPower);
        t.addData("Shooter Distance (pose units)", currentDistanceM);
        t.addData("Shooter LUT Distance (pose units)", dist);
        t.addData("Shooter Velocity", getVelocity());
        t.addData("Shooter Target Velocity", getTargetVelocity());
        t.addData("Shooter Hood LUT (deg)", currentTargetHoodAngle);
        t.addData("Shooter Hood Cmd (deg)", hoodCmdDeg);
        t.addData("Shooter Ready", readyToShoot());
    }
}
