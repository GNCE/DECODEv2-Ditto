package org.firstinspires.ftc.teamcode.config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
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

/**
 * Distance-based shooter using LUTs:
 *  - InterpLUT: distance (m) -> flywheel velocity (ticks/sec)
 *  - InterpLUT: distance (m) -> hood angle (deg from vertical)
 *
 * All empirical behavior (backspin, slip, etc.) is baked into the LUT data.
 */
@Configurable
public class Shooter extends SubsysCore {

    // ===========================
    // Hardware
    // ===========================
    private final MotorGroup flywheel;
    private final Motor.Encoder encoder;
    private final Servo hood;

    // ===========================
    // Motor / control parameters
    // ===========================

    // We control directly in flywheel.getVelocity() units: motor ticks/sec.

    // Idle velocity when inactive
    public static double INACTIVE_VELOCITY = 0.0;          // ticks/sec

    // Max allowed velocity (safety clamp)
    // 6000 rpm motor -> 6000/60 * 28 = 2800 ticks/sec.
    // We add +100 as a small headroom margin.

    // How close (in ticks/sec) must we be to call readyToShoot()
    public static double VELOCITY_READY_THRESHOLD = 70.0;


    // PID gains (live-tunable)
    public static double kp = 0.0006;
    public static double ki = 0.0;
    public static double kd = 0;
    public static double kV = 0.0003912;

    // Last commanded target velocity (ticks/sec)
    private double currentTargetVelocity = 0.0;
    private double currentTargetHoodAngle = 30;

    // ===========================
    // Geometry (metric)
    // ===========================

    // Horizontal offset: robot center → hood center, along robot forward (m)

    // Launcher nominal height (ball center) (m)
    public static double LAUNCHER_HEIGHT_M = 0.26839625; // 268.39625 mm

    // Hood angle range (deg FROM VERTICAL: 0° up, 90° horizontal)
    public static double MIN_HOOD_ANGLE_DEG = 36; // was 28
    public static double MAX_HOOD_ANGLE_DEG = 55.0;

    // Servo positions at angle limits
    public static double HOOD_MAX_SERVO_POS = 0.42; // servo tester on the right, hood lowest. higher pos = lower
    public static double HOOD_GEAR_RATIO =  (double)300/44;
    public static double HOOD_SERVO_RANGE = 355.0;

    // Hood angle when inactive (stowed)
    public static double IDLE_HOOD_ANGLE_DEG = MAX_HOOD_ANGLE_DEG;

    // ===========================
    // LUTs: distance → velocity, distance → hood angle
    //
    // Distances in meters (hood to goal)
    // Velocity in motor ticks/sec (flywheel.getVelocity() units)
    // Hood angle in degrees FROM VERTICAL
    //
    // TUNE THESE ARRAYS WITH REAL DATA.
    // ===========================

    private final double[] distances = {
            49.7, 69.5
    };
    private final double[] velocities = {
            1900, 2100,
    };

    private final double[] hoodAngles = {
            36, 38,
    };
    PIDFController pidfController;

    private final InterpLUT velocityLut = new InterpLUT();
    private final InterpLUT hoodAngleLut = new InterpLUT();

    // ===========================
    // State
    // ===========================

    // Latest distance (m) from hood to goal (updated externally each loop)
    private double currentDistanceM = 0.0;

    // Whether shooter is actively tracking / shooting
    public boolean active = true;

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
    }

    // ===========================
    // Public API
    // ===========================

    public void setActive(boolean active) {
        this.active = active;
    }

    public boolean isActive() {
        return active;
    }

    public double getTargetVelocity(){
        return currentTargetVelocity;
    }


    /** Simple “on target” check in velocity units. */
    public boolean readyToShoot() {
        if (!active) return false;

        return Math.abs(getTargetVelocity() - getVelocity()) < VELOCITY_READY_THRESHOLD;
    }

    /**
     * Compute horizontal distance (m) from hood to goal using Pedro Poses.
     *
     * Assumptions:
     *  - robotPose & goalPose x,y are in meters
     *  - heading in radians; 0 rad is +X, CCW positive
     *  - hood is TURRET_OFFSET_M in front of robot center along heading
     */

    public void input(Pose robotPose, Pose goalPose) {
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        currentDistanceM = Math.hypot(dx, dy);
    }

    // ===========================
    // Internal helpers
    // ===========================

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
        return Math.abs(encoder.getCorrectedVelocity());
    }
    public void setHoodAngle(double angle){
        currentTargetHoodAngle = angle;
    }

    public static boolean autoTarget = true;
    public static double NOMINAL_VOLTAGE = 13.5;

    @Override
    public void periodic() {
        // Allow PID tuning
        pidfController.setPIDF(kp, ki, kd, kV);

        if (active) {
            if(autoTarget){
                // 1) Lookup target velocity and angle from distance using LUTs
                double dist = MathUtils.clamp(currentDistanceM, distances[0]*1.01, distances[distances.length-1]*0.99);
                double lutVelocity   = velocityLut.get(dist);
                double lutHoodAngle  = hoodAngleLut.get(dist);

                // 2) Clamp velocity and enforce hood angle limits
                currentTargetVelocity   = Range.clip(lutVelocity, 0.0, 1.0);
                currentTargetHoodAngle = Range.clip(lutHoodAngle, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
            } else {

            }
        } else {
            // Inactive: idle behavior
            currentTargetVelocity   = INACTIVE_VELOCITY;
            currentTargetHoodAngle = IDLE_HOOD_ANGLE_DEG;
        }


        // Command flywheel directly in ticks/sec
        flywheel.set(MathUtils.clamp(pidfController.calculate(getVelocity(), getTargetVelocity()), -1, 1));

        // Command hood servo
        double servoPos = hoodAngleToServoPos(currentTargetHoodAngle);
        hood.setPosition(servoPos);

        t.addData("Shooter Velocity Error", getTargetVelocity() - getVelocity());
        t.addData("Shooter Distance (M)", currentDistanceM);
        t.addData("Shooter Velocity", getVelocity());
        t.addData("Shooter Target Velocity", getTargetVelocity());
        t.addData("Shooter Ready", readyToShoot());
    }
}
