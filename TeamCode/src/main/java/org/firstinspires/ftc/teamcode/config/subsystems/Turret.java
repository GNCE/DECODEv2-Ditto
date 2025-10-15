package org.firstinspires.ftc.teamcode.config.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.core.SubsysCore;

public class Turret extends SubsysCore {
    CRServo s1, s2;
    AbsoluteAnalogEncoder e1, e2;
    Follower f;
    Limelight ll;
    PIDController pid;
    public static double kp = 0.5, ki = 0.00001, kd = 0.05;

    double turretDeg; // 0 is the position where the shooter is facing the back. This is NOT the actual turret angle.
    final double GEAR_RATIO = (double) 180 /50 * 20 / 90;

    double previousServoAngle;
    double previousPower;
    private final Pose goal;

    public double servoToTurretDegrees(double deg){
        return deg/GEAR_RATIO;
    }
    public double turretToServoDegrees(double deg){
        return deg*GEAR_RATIO;
    }

    public Turret(Follower follower, Limelight ll, double initialTurretDeg,  boolean isRed){ // %TODO: isRed should not be here. It should be able to change during initialization
        this.ll = ll;
        e1 = new AbsoluteAnalogEncoder(h, "te1", 3.3, AngleUnit.DEGREES);
        s1 = h.get(CRServo.class, "ts1");
        s2 = h.get(CRServo.class, "ts2");

        turretDeg = initialTurretDeg - servoToTurretDegrees(e1.getCurrentPosition());
        previousServoAngle = e1.getCurrentPosition();
        previousPower = 0;
        f = follower;
        goal = isRed ? new Pose(132.5, 138): new Pose(11.5, 138);
        pid = new PIDController(kp, ki, kd);
    }


    double getCurrentTurretAngle(){
        return turretDeg + servoToTurretDegrees(e1.getCurrentPosition());
    }

    double turretTargetDegFromPinpoint(){
        double turretTarget = Math.toDegrees(Math.atan2(goal.getY() - f.getPose().getY(), goal.getX() - f.getPose().getX()) - f.getPose().getHeading() - Math.PI);
        turretTarget = (turretTarget + 540)%360-180;
        return turretTarget;
    }

    @Override
    public void periodic() {

        double currentServoAngle = e1.getCurrentPosition();
        if(previousPower > 0 && previousServoAngle > 240 && currentServoAngle < 120) turretDeg += servoToTurretDegrees(360);
        else if(previousPower < 0 && previousServoAngle < 120 && currentServoAngle > 240) turretDeg -= servoToTurretDegrees(360);

        pid.setPID(kp, ki, kd);

        double err;
        if(ll.isAprilTagDetected()){
            err = 0; // TODO: Do this
        } else {
            err = turretTargetDegFromPinpoint() - getCurrentTurretAngle();
        }

        double pwr = pid.calculate(0, err);
        s1.setPower(pwr);
        s2.setPower(pwr);

        previousServoAngle = e1.getCurrentPosition();
    }

    public boolean reachedTarget(){ return true; }
}
