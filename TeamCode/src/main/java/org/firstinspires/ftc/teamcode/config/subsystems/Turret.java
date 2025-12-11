package org.firstinspires.ftc.teamcode.config.subsystems;

import androidx.core.math.MathUtils;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.core.SubsysCore;

@Configurable
public class Turret extends SubsysCore {
    CRServo s1, s2;
    AbsoluteAnalogEncoder e1, e2;
    Follower f;
    Limelight ll;
    PIDController pid;
    public static double kp = 0.005, ki = 0, kd = 0.00005, kF = 0.1;
    public static double REACHED_TARGET_THRESHOLD = 2;
    public static double ZERO_OFFSET = -12;

    double turretDeg; // 0 is the position where the shooter is facing the back. This is NOT the actual turret angle.
    final double GEAR_RATIO = (double) 180 /50 * 20 / 90;

    double previousServoAngle, currentServoAngle;
    double previousPower;
    public static int wrapCount;
    double err;
    private Pose goalPose, robotPose;

    public double servoToTurretDegrees(double deg){
        return deg/GEAR_RATIO;
    }
    public double turretToServoDegrees(double deg){
        return deg*GEAR_RATIO;
    }

    public Turret(int initialWrap){ // %TODO: isRed should not be here. It should be able to change during initialization
        e1 = new AbsoluteAnalogEncoder(h, "te1", 3.3, AngleUnit.DEGREES);
        s1 = h.get(CRServo.class, "ts1");
        s2 = h.get(CRServo.class, "ts2");
        s1.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setDirection(DcMotorSimple.Direction.REVERSE);

        wrapCount = initialWrap;
        previousServoAngle = e1.getCurrentPosition();
        previousPower = 0;
        pid = new PIDController(kp, ki, kd);
    }


    double getCurrentTurretAngle(){
        return servoToTurretDegrees(wrapCount*360+currentServoAngle+ZERO_OFFSET);
    }

    double turretTargetDegFromPinpoint(){
        double turretTarget = Math.toDegrees(Math.atan2(goalPose.getY() - robotPose.getY(), goalPose.getX() - robotPose.getX()) - robotPose.getHeading() - Math.PI);
        turretTarget = (turretTarget + 540)%360-180;
        return turretTarget;
    }

    public static double targetTurret = 0;

    public void input(Pose robotPose, Pose goalPose){
        this.robotPose = robotPose;
        this.goalPose = goalPose;
    }

    @Override
    public void periodic() {
        currentServoAngle = e1.getCurrentPosition();
        double deltaAngle = currentServoAngle - previousServoAngle;
        if(deltaAngle < -180) wrapCount++;
        else if (deltaAngle > 180) wrapCount--;

        pid.setPID(kp, ki, kd);

        targetTurret = turretTargetDegFromPinpoint();
/*
        if(ll.isAprilTagDetected()){
            err = 0; // TODO: Do this
        } else {
            targetTurret = turretTargetDegFromPinpoint();
        }

 */
        targetTurret = MathUtils.clamp(targetTurret, -180, 180);
        err = targetTurret - getCurrentTurretAngle();

        double calc = pid.calculate(0, err);
        if(err > 0) calc += kF;
        else if(err < 0) calc -= kF;
        double pwr = MathUtils.clamp(calc, -1, 1);
        s1.setPower(pwr);
        s2.setPower(pwr);

        t.addData("Turret Wrap Count", wrapCount);
        t.addData("Turret Current Power", pwr);
        t.addData("Turret Target Angle", targetTurret);
        t.addData("Turret Target Pinpoint", turretTargetDegFromPinpoint());
        t.addData("Turret Current Angle", getCurrentTurretAngle());
        t.addData("Turret Servo Angle", currentServoAngle);
        previousServoAngle = currentServoAngle;
    }

    public boolean reachedTarget(){ return Math.abs(err) < REACHED_TARGET_THRESHOLD; }

    public int getWrapCount() {
        return wrapCount;
    }
}
