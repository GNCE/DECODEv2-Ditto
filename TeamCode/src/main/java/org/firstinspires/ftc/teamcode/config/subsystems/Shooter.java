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
import org.firstinspires.ftc.teamcode.config.core.util.ShotPlanner;
import org.firstinspires.ftc.teamcode.config.core.util.hardware.VoltageCompensatedMotorGroup;

@Configurable
public class Shooter extends SubsysCore {
    private final VoltageCompensatedMotorGroup flywheel;
    private final Motor.Encoder encoder;
    private final Servo hood;

    public static double INACTIVE_VELOCITY = 0.0;
    public static double VELOCITY_READY_THRESHOLD = 60.0;

    public static double kp = 0.01;
    public static double ki = 0.0;
    public static double kd = 0;
    public static double kV = 0.000455;

    private double currentTargetVelocity = 0.0;
    private double currentTargetHoodAngle = 30;

    public static double MIN_HOOD_ANGLE_DEG = 30;
    public static double MAX_HOOD_ANGLE_DEG = 55.5;

    public static double HOOD_MAX_SERVO_POS = 1;
    public static double HOOD_GEAR_RATIO = (double) 300 / 44;
    public static double HOOD_SERVO_RANGE = 255;
    public static double IDLE_HOOD_ANGLE_DEG = 31;

    PIDFController pidfController;
    boolean active = false;

    public static boolean testModeOnly = false;

    private double plannedDistPoseUnits = 0.0;        // inches
    private double plannedTargetRpm = 0.0;            // rpm
    private double plannedHoodBaselineDeg = IDLE_HOOD_ANGLE_DEG; // hood deg (your convention)
    private boolean shotPossible = false;

    public void setPlannedShot(double distPoseUnits, double targetRpm, double hoodBaselineDegFromVertical, boolean plannedPossible) {
        plannedDistPoseUnits = distPoseUnits;
        plannedTargetRpm = targetRpm;
        plannedHoodBaselineDeg = hoodBaselineDegFromVertical;
        shotPossible = plannedPossible;
    }

    public static double NOMINAL_VOLTAGE = 12.0;

    public Shooter() {
        Motor m1 = new MotorEx(h, "shooter1", Motor.GoBILDA.BARE), m2 = new MotorEx(h, "shooter2", Motor.GoBILDA.BARE);
        m2.setInverted(true);
        flywheel = new VoltageCompensatedMotorGroup(h, 500L, NOMINAL_VOLTAGE, m1, m2);
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
        return shotPossible;
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
            currentTargetHoodAngle = plannedHoodBaselineDeg;
        }

        double targetPower = MathUtils.clamp(pidfController.calculate(getVelocity(), getTargetVelocity()), -1, 1);
        flywheel.set(targetPower);

        // Physics comp removed — hood angle from ShotPlanner is already physics-correct
        // double hoodCmdDeg = (active && !testModeOnly)
        //         ? physicsCompensatedHoodDeg(plannedDistPoseUnits, currentTargetVelocity, getVelocity(), currentTargetHoodAngle)
        //         : currentTargetHoodAngle;
        double hoodCmdDeg = currentTargetHoodAngle;

        double servoPos = hoodAngleToServoPos(hoodCmdDeg);
        hood.setPosition(shotPossible ? servoPos : IDLE_HOOD_ANGLE_DEG);

        t.addData("Shooter Cached Voltage", flywheel.getCachedVoltage());
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