package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.core.SubsysCore;

public class Turret extends SubsysCore {
    CRServo s1, s2;
    AbsoluteAnalogEncoder e1, e2;

    int fw;
    final double GEAR_RATIO = (double) 180 /50 * 20 / 90;

    public double servoToTurretDegrees(){  }
    public double turretToServoDegrees(){ }

    public Turret(){
        e1 = new AbsoluteAnalogEncoder(h, "te1", 3.3, AngleUnit.DEGREES);
    }
    public boolean reachedTarget(){ return true; }
}
