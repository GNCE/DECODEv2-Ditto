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
    public static double kV = 0.000352;
    private double currentTargetVelocity = 0.0;
    private double currentTargetHoodAngle = 30;
    public static double LAUNCHER_HEIGHT_M = 0.26839625; // 268.39625 mm

    // Hood angle range (deg FROM VERTICAL: 0° up, 90° horizontal)
    public static double MIN_HOOD_ANGLE_DEG = 30; // was 28
    public static double MAX_HOOD_ANGLE_DEG = 55.0;

    // Servo positions at angle limits
    public static double HOOD_MAX_SERVO_POS = 0.595; // servo tester on the right, hood lowest. higher pos = lower
    public static double HOOD_GEAR_RATIO =  (double)300/44;
    public static double HOOD_SERVO_RANGE = 355.0;
    public static double IDLE_HOOD_ANGLE_DEG = MAX_HOOD_ANGLE_DEG;
    private final double[] distances = {
            22.4, 42.9, 57.425, 72.467, 85.67, 117.3, 130.3
    };
    private final double[] velocities = {
            2050, 2100, 2180, 2320, 2500, 2800, 2900
    };
    private final double[] hoodAngles = {
            40, 48, 53, 53, 55, 53, 50
    };
    PIDFController pidfController;
    private final InterpLUT velocityLut = new InterpLUT();
    private final InterpLUT hoodAngleLut = new InterpLUT();
    private double currentDistanceM = 0.0;
    boolean active = true;
    public Shooter() {
        Motor m1 = new MotorEx(h, "shooter1", Motor.GoBILDA.BARE), m2 = new MotorEx(h, "shooter2", Motor.GoBILDA.BARE);
        m1.setInverted(true);
        flywheel = new MotorGroup(m1, m2);
        flywheel.setRunMode(Motor.RunMode.RawPower);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        encoder = m1.encoder;
        encoder.setDirection(Motor.Direction.FORWARD); // apparently does not do anything

        hood = h.get(Servo.class, "hood");

        // Build interpolation once
        for(int i=0; i<distances.length; i++){
            velocityLut.add(distances[i], velocities[i]);
            hoodAngleLut.add(distances[i], hoodAngles[i]);
        }
        velocityLut.createLUT();
        hoodAngleLut.createLUT();
        pidfController = new PIDFController(kp, ki, kd, kV);

        setDefaultCommand(new RunCommand(() -> active = true, this));
    }
    public void setActive(boolean active) {
        this.active = active;
    }
    public boolean isActive() {
        return active;
    }
    public double getTargetVelocity(){
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
        double clipped = Range.clip(
                hoodAngleDeg,
                MIN_HOOD_ANGLE_DEG,
                MAX_HOOD_ANGLE_DEG
        );

        double t = (MAX_HOOD_ANGLE_DEG - clipped);
        return HOOD_MAX_SERVO_POS +
                t * HOOD_GEAR_RATIO/HOOD_SERVO_RANGE;
    }

    public void setVelocity(double vel){
        currentTargetVelocity = vel;
    }

    public double getVelocity(){
        return -encoder.getCorrectedVelocity();
    }
    public void setHoodAngle(double angle){
        currentTargetHoodAngle = angle;
    }

    public static double NOMINAL_VOLTAGE = 13.5;

    @Override
    public void periodic() {
        pidfController.setPIDF(kp, ki, kd, kV);

        if (active) {
            double dist = MathUtils.clamp(currentDistanceM, distances[0]*1.01, distances[distances.length-1]*0.99);
            double lutVelocity   = velocityLut.get(dist);
            double lutHoodAngle  = hoodAngleLut.get(dist);
            currentTargetVelocity   = Range.clip(lutVelocity, 0.0, 2800);
            currentTargetHoodAngle = Range.clip(lutHoodAngle, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        } else {
            currentTargetVelocity = INACTIVE_VELOCITY;
            currentTargetHoodAngle = IDLE_HOOD_ANGLE_DEG;
        }

        double targetPower = MathUtils.clamp(pidfController.calculate(getVelocity(), getTargetVelocity()), -1, 1);
        flywheel.set(targetPower);

        double servoPos = hoodAngleToServoPos(currentTargetHoodAngle);
        hood.setPosition(servoPos);

        t.addData("Shooter Velocity Error", getTargetVelocity() - getVelocity());
        t.addData("Shooter Target Power", targetPower);
        t.addData("Shooter Distance (M)", currentDistanceM);
        t.addData("Shooter Velocity", getVelocity());
        t.addData("Shooter Target Velocity", getTargetVelocity());
        t.addData("Shooter Ready", readyToShoot());
    }
}
