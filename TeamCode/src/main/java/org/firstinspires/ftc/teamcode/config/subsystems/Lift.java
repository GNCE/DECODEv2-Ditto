package org.firstinspires.ftc.teamcode.config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.SubsysCore;

@Configurable
public class Lift extends SubsysCore {
    DcMotorEx l;
    public static int tar = 0;


    public enum LiftPositions {
        RETRACTED(0), EXTENDED(11000);

        private final int ticks;
        LiftPositions(int ticks){
            this.ticks = ticks;
        }
    }

    public Lift(){
        l = h.get(DcMotorEx.class, "liftMotor");
        l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setMode(Mode.RTP);
        setTargetPosition(LiftPositions.RETRACTED);
    }

    public void setTargetPosition(LiftPositions liftPositions){
        tar = liftPositions.ticks;
    }

    public enum Mode {
        MANUAL,
        RTP
    }

    Mode mode;

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    double pwr = 0;

    public void setManualPower(double pwr){
        this.pwr = pwr;
    }

    @Override
    public void periodic() {
        switch (mode){
            case RTP:
                l.setTargetPosition(tar);
                l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(Math.abs(tar - l.getCurrentPosition()) < 100) l.setPower(0);
                else l.setPower(1);

                t.addData("Lift Target Position", tar);
                break;
            case MANUAL:
                l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                l.setPower(pwr);
                break;
        }
        t.addData("Lift Current Position", l.getCurrentPosition());
        t.addData("Lift Velocity", l.getVelocity());
        t.addData("Lift Current Amps", l.getCurrent(CurrentUnit.AMPS));
        t.addData("Lift Busy", l.isBusy());
    }

    public boolean reachedTarget(){
        return !l.isBusy();
    }
}
