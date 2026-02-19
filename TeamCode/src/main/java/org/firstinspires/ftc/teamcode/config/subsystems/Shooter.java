package org.firstinspires.ftc.teamcode.config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;

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

    public static double MIN_HOOD_ANGLE_DEG = 29;
    public static double MAX_HOOD_ANGLE_DEG = 52.514;

    public static double HOOD_MAX_SERVO_POS = 0.9;
    public static double HOOD_GEAR_RATIO = (double) 300 / 44;
    public static double HOOD_SERVO_RANGE = 355.0;
    public static double IDLE_HOOD_ANGLE_DEG = MAX_HOOD_ANGLE_DEG;

    PIDFController pidfController;
    boolean active = false;

    public static boolean testModeOnly = false;

    private double plannedDistPoseUnits = 0.0;        // inches
    private double plannedTargetRpm = 0.0;            // rpm
    private double plannedHoodBaselineDeg = IDLE_HOOD_ANGLE_DEG; // hood deg (your convention)

    public void setPlannedShot(double distPoseUnits, double targetRpm, double hoodBaselineDegFromVertical) {
        plannedDistPoseUnits = distPoseUnits;
        plannedTargetRpm = targetRpm;
        plannedHoodBaselineDeg = hoodBaselineDegFromVertical;
    }

    public static double NOMINAL_VOLTAGE = 13.5;

    public static double POSE_UNITS_TO_METERS = 0.0254;

    public static double GOAL_HEIGHT_M = 1.14;
    public static double LAUNCHER_HEIGHT_ACTUAL_M = 0.38;

    public static double GRAVITY = 9.80665;

    public static double EXIT_VEL_M_PER_RPM = 0.0039;

    public static double PHYS_COMP_MAX_DELTA_DEG = 6.0;
    public static double PHYS_COMP_MIN_FRAC = 0.55;
    public static boolean PHYS_COMP_ENABLED = true;

    private double hoodToThetaRad(double hoodDeg) {
        return Math.toRadians(90.0 - hoodDeg);
    }

    private double thetaToHoodDeg(double thetaRadFromHorizontal) {
        return 90.0 - Math.toDegrees(thetaRadFromHorizontal);
    }

    private static class ThetaPair {
        final double thLo;
        final double thHi;
        ThetaPair(double thLo, double thHi) { this.thLo = thLo; this.thHi = thHi; }
    }

    private ThetaPair solveThetaPair(double x, double v, double dh) {
        if (x <= 1e-6 || v <= 1e-6) return null;

        double g = GRAVITY;
        double v2 = v * v;
        double v4 = v2 * v2;

        double disc = v4 - g * (g * x * x + 2.0 * dh * v2);
        if (disc < 0.0) return null;

        double sqrt = Math.sqrt(disc);

        double tanHi = (v2 + sqrt) / (g * x);
        double tanLo = (v2 - sqrt) / (g * x);

        double thHi = Math.atan(tanHi);
        double thLo = Math.atan(tanLo);

        return new ThetaPair(thLo, thHi);
    }

    private boolean wouldClip(double hoodDeg) {
        return hoodDeg < MIN_HOOD_ANGLE_DEG || hoodDeg > MAX_HOOD_ANGLE_DEG;
    }

    private double clipAmount(double hoodDeg) {
        if (hoodDeg < MIN_HOOD_ANGLE_DEG) return MIN_HOOD_ANGLE_DEG - hoodDeg;
        if (hoodDeg > MAX_HOOD_ANGLE_DEG) return hoodDeg - MAX_HOOD_ANGLE_DEG;
        return 0.0;
    }

    private double physicsCompensatedHoodDeg(double distPoseUnits,
                                             double targetRPM,
                                             double actualRPM,
                                             double hoodBaselineDeg) {

        if (!PHYS_COMP_ENABLED) {
            return Range.clip(hoodBaselineDeg, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        }

        double tgt = Math.abs(targetRPM);
        double act = Math.abs(actualRPM);

        if (tgt < 1.0) {
            return Range.clip(hoodBaselineDeg, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        }
        if (act < PHYS_COMP_MIN_FRAC * tgt) {
            return Range.clip(hoodBaselineDeg, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        }

        double x = distPoseUnits * POSE_UNITS_TO_METERS;
        double dh = GOAL_HEIGHT_M - LAUNCHER_HEIGHT_ACTUAL_M;

        double vTarget = tgt * EXIT_VEL_M_PER_RPM;
        double vActual = act * EXIT_VEL_M_PER_RPM;

        ThetaPair pairT = solveThetaPair(x, vTarget, dh);
        ThetaPair pairA = solveThetaPair(x, vActual, dh);

        if (pairT == null || pairA == null) {
            return Range.clip(hoodBaselineDeg, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        }

        // Compute hoodCmd for LO branch
        double hoodTargetLo = thetaToHoodDeg(pairT.thLo);
        double hoodActualLo = thetaToHoodDeg(pairA.thLo);
        double deltaLo = Range.clip(hoodActualLo - hoodTargetLo, -PHYS_COMP_MAX_DELTA_DEG, PHYS_COMP_MAX_DELTA_DEG);
        double hoodCmdLo = hoodBaselineDeg + deltaLo;

        // Compute hoodCmd for HI branch
        double hoodTargetHi = thetaToHoodDeg(pairT.thHi);
        double hoodActualHi = thetaToHoodDeg(pairA.thHi);
        double deltaHi = Range.clip(hoodActualHi - hoodTargetHi, -PHYS_COMP_MAX_DELTA_DEG, PHYS_COMP_MAX_DELTA_DEG);
        double hoodCmdHi = hoodBaselineDeg + deltaHi;

        boolean clipLo = wouldClip(hoodCmdLo);
        boolean clipHi = wouldClip(hoodCmdHi);

        // YOUR RULE:
        // Choose the other branch if one branch clips.
        double hoodCmd;
        if (clipLo && !clipHi) hoodCmd = hoodCmdHi;
        else if (clipHi && !clipLo) hoodCmd = hoodCmdLo;
        else if (!clipLo && !clipHi) {
            // neither clips: pick the branch whose TARGET solution is closer to LUT baseline (follows LUT family)
            double aLo = Math.abs(hoodTargetLo - hoodBaselineDeg);
            double aHi = Math.abs(hoodTargetHi - hoodBaselineDeg);
            hoodCmd = (aHi <= aLo) ? hoodCmdHi : hoodCmdLo;
        } else {
            // both clip: pick smaller clip amount
            hoodCmd = (clipAmount(hoodCmdLo) <= clipAmount(hoodCmdHi)) ? hoodCmdLo : hoodCmdHi;
        }

        t.addData("Hood Target Calc", thetaToHoodDeg((!clipLo && clipHi) ? pairT.thLo : (!clipHi && clipLo) ? pairT.thHi : pairT.thLo));
        t.addData("Hood Actual Calc", thetaToHoodDeg((!clipLo && clipHi) ? pairA.thLo : (!clipHi && clipLo) ? pairA.thHi : pairA.thLo));
        t.addData("Hood LUT", hoodBaselineDeg);
        t.addData("HoodCmdLo", hoodCmdLo);
        t.addData("HoodCmdHi", hoodCmdHi);
        t.addData("ClipLo", clipLo);
        t.addData("ClipHi", clipHi);

        return Range.clip(hoodCmd, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
    }

    public Shooter() {
        Motor m1 = new MotorEx(h, "shooter1", Motor.GoBILDA.BARE), m2 = new MotorEx(h, "shooter2", Motor.GoBILDA.BARE);
        m1.setInverted(true);
        flywheel = new MotorGroup(m1, m2);
        flywheel.setRunMode(Motor.RunMode.RawPower);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        encoder = m1.encoder;
        encoder.setDirection(Motor.Direction.FORWARD);

        hood = h.get(Servo.class, "hood");

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

        double servoPos = hoodAngleToServoPos(hoodCmdDeg);
        hood.setPosition(servoPos);

        t.addData("Shooter Velocity Error", getTargetVelocity() - getVelocity());
        t.addData("Shooter Target Power", targetPower);
        t.addData("Shooter Velocity", getVelocity());
        t.addData("Shooter Target Velocity", getTargetVelocity());
        t.addData("Shooter Ready", readyToShoot());
        t.addData("Shooter testModeOnly", testModeOnly);
        t.addData("Shooter Planned Dist (in)", plannedDistPoseUnits);
        t.addData("Shooter Hood Baseline (deg)", currentTargetHoodAngle);
        t.addData("Shooter Hood Cmd (deg)", hoodCmdDeg);
        t.addData("Shooter Commanded Servo Pos", servoPos);
    }
}
