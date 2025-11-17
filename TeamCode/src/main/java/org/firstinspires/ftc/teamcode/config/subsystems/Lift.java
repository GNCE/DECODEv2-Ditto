package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.core.SubsysCore;

public class Lift extends SubsysCore {
    DcMotorEx l;
    int tar = 0;

    public enum LiftPositions {
        RETRACTED(0), EXTENDED(100);

        private final int ticks;
        LiftPositions(int ticks){
            this.ticks = ticks;
        }
    }

    public Lift(){
        l = h.get(DcMotorEx.class, "liftMotor");
        l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        l.setTargetPositionTolerance(20);
        setTargetPosition(LiftPositions.RETRACTED);
    }

    public void setTargetPosition(LiftPositions liftPositions){
        tar = liftPositions.ticks;
    }

    @Override
    public void periodic() {
        l.setTargetPosition(tar);
        t.addData("Lift Current Position", l.getCurrentPosition());
        t.addData("Lift Current Amps", l.getCurrent(CurrentUnit.AMPS));
        t.addData("Lift Busy", l.isBusy());
    }

    public boolean reachedTarget(){
        return !l.isBusy();
    }
}
