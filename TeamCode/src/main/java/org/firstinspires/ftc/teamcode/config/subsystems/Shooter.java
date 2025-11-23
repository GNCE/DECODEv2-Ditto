package org.firstinspires.ftc.teamcode.config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import com.pedropathing.geometry.Pose;

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
    public static double MAX_VELOCITY = 2800.0 + 100.0;

    // How close (in ticks/sec) must we be to call readyToShoot()
    public static double VELOCITY_READY_THRESHOLD = 50.0;


    // PID gains (live-tunable)
    public static double kp = 0.5;
    public static double ki = 0.0;
    public static double kd = 0.05;

    // Last commanded target velocity (ticks/sec)
    private double currentTargetVelocity = 0.0;

    // ===========================
    // Geometry (metric)
    // ===========================

    // Horizontal offset: robot center → hood center, along robot forward (m)
    public static double TURRET_OFFSET_M = 0.08288647; // 82.88647 mm

    // Launcher nominal height (ball center) (m)
    public static double LAUNCHER_HEIGHT_M = 0.26839625; // 268.39625 mm

    // Hood angle range (deg FROM VERTICAL: 0° up, 90° horizontal)
    public static double MIN_HOOD_ANGLE_DEG = 28.0;
    public static double MAX_HOOD_ANGLE_DEG = 55.0;

    // Servo positions at angle limits
    public static double HOOD_MIN_SERVO_POS = 0.20;  // at MIN_HOOD_ANGLE_DEG
    public static double HOOD_GEAR_RATIO =  (double)532/110;
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
            1.0, 2.0, 3.0
    };
    private final double[] velocities = {
            2000.0, 2400.0, 2800.0
    };

    private final double[] hoodAngles = {
            50.0, 45.0, 40.0
    };

    private final InterpLUT velocityLut = new InterpLUT();
    private final InterpLUT hoodAngleLut = new InterpLUT();

    // ===========================
    // State
    // ===========================

    // Latest distance (m) from hood to goal (updated externally each loop)
    private double currentDistanceM = 0.0;

    // Whether shooter is actively tracking / shooting
    private boolean active = false;

    public Shooter() {
        flywheel = new MotorGroup(
                new Motor(h, "shooter1", Motor.GoBILDA.BARE),
                new Motor(h, "shooter2", Motor.GoBILDA.BARE)
        );
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        hood = h.get(Servo.class, "hood");

        // Build interpolation once
        for(int i=0; i<distances.length; i++){
            velocityLut.add(distances[i], velocities[i]);
            hoodAngleLut.add(distances[i], hoodAngles[i]);
        }
        velocityLut.createLUT();
        hoodAngleLut.createLUT();
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

    /** Distance from hood to goal (meters), call this every loop. */
    public void setDistance(double distanceM) {
        this.currentDistanceM = distanceM;
    }

    public double getDistance() {
        return currentDistanceM;
    }

    /** Simple “on target” check in velocity units. */
    public boolean readyToShoot() {
        if (!active) return false;

        double currentVelo = flywheel.getVelocity();
        return Math.abs(currentVelo - currentTargetVelocity) < VELOCITY_READY_THRESHOLD;
    }

    /**
     * Compute horizontal distance (m) from hood to goal using Pedro Poses.
     *
     * Assumptions:
     *  - robotPose & goalPose x,y are in meters
     *  - heading in radians; 0 rad is +X, CCW positive
     *  - hood is TURRET_OFFSET_M in front of robot center along heading
     */
    public static double distanceFromHoodToGoal(Pose robotPose, Pose goalPose) {
        double xRobot = robotPose.getX();
        double yRobot = robotPose.getY();
        double heading = robotPose.getHeading(); // rad

        double xHood = xRobot + TURRET_OFFSET_M * Math.cos(heading);
        double yHood = yRobot + TURRET_OFFSET_M * Math.sin(heading);

        double dx = goalPose.getX() - xHood;
        double dy = goalPose.getY() - yHood;

        return Math.hypot(dx, dy);
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

        double t = (clipped - MIN_HOOD_ANGLE_DEG) /
                (MAX_HOOD_ANGLE_DEG - MIN_HOOD_ANGLE_DEG);
        return HOOD_MIN_SERVO_POS +
                t * HOOD_GEAR_RATIO/HOOD_SERVO_RANGE;
    }

    // ===========================
    // periodic(): apply LUT-based targets
    // ===========================

    @Override
    public void periodic() {
        // Allow PID tuning
        flywheel.setVeloCoefficients(kp, ki, kd);

        double veloTarget;
        double hoodAngleDeg;

        if (active) {
            // 1) Lookup target velocity and angle from distance using LUTs
            double lutVelocity   = velocityLut.get(currentDistanceM);
            double lutHoodAngle  = hoodAngleLut.get(currentDistanceM);

            // 2) Clamp velocity and enforce hood angle limits
            veloTarget   = Range.clip(lutVelocity, 0.0, MAX_VELOCITY);
            hoodAngleDeg = Range.clip(lutHoodAngle, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        } else {
            // Inactive: idle behavior
            veloTarget   = INACTIVE_VELOCITY;
            hoodAngleDeg = IDLE_HOOD_ANGLE_DEG;
        }

        // Cache for readyToShoot()
        currentTargetVelocity = veloTarget;

        // Command flywheel directly in ticks/sec
        flywheel.set(veloTarget);

        // Command hood servo
        double servoPos = hoodAngleToServoPos(hoodAngleDeg);
        hood.setPosition(servoPos);
    }
}
