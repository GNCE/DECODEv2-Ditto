package org.firstinspires.ftc.teamcode.config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;

import kotlin.time.Instant;

@Configurable
public class Lift extends SubsysCore {
    MotorGroup front, back;
    ServoEx pto;
    public static int tar = 0;
    public static double kp = 0, ki = 0, kd = 0, kf = 0;
    public static int positionTolerance = 50;
    public static double CLUTCH_PULLED = 0;
    public static double CLUTCH_MESHED = 1;
    public static double ENGAGING_POWER = 0.3;
    public static long ENGAGING_DELAY = 1000;
    PIDFController pidf;


    public enum LiftPositions {
        RETRACTED(0), EXTENDED(11000);

        private final int ticks;
        LiftPositions(int ticks){
            this.ticks = ticks;
        }
    }

    public Lift(){
        front = new MotorGroup(
                new MotorEx(h, "fl"),
                new MotorEx(h, "fr").setInverted(true)
        );
        back = new MotorGroup(
                new MotorEx(h, "bl"),
                new MotorEx(h, "br").setInverted(true)
        );
        pto = new ServoEx(h, "pto");
        back.setRunMode(Motor.RunMode.RawPower);
        setMode(Mode.INACTIVE);
        setTargetPosition(LiftPositions.RETRACTED);
        pidf = new PIDFController(kp, ki, kd, kf);
        pidf.setTolerance(positionTolerance);
    }

    public void setTargetPosition(LiftPositions liftPositions){
        tar = liftPositions.ticks;
    }

    public enum Mode {
        RAW,
        PID,
        INACTIVE
    }

    Mode mode;

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    double pwr = 0;

    public void setManualPower(double pwr){
        this.pwr = pwr;
    }
    public int getCurrentPosition(){
        return back.getCurrentPosition();
    }

    @Override
    public void periodic() {
        switch (mode){
            case PID:
                pidf.setPIDF(kp, ki, kd, 0);
                pidf.setTolerance(positionTolerance);
                pidf.setSetPoint(tar);
                pwr = MathUtils.clamp(pidf.calculate(getCurrentPosition()) + kf, -1, 1);
                back.set(pwr);
                t.addData("Lift Target Position", tar);
                break;
            case RAW:
                front.set(-pwr);
                back.set(pwr);
                break;
        }
        if(mode == Mode.INACTIVE) pto.set(CLUTCH_PULLED);
        else pto.set(CLUTCH_MESHED);

        t.addData("Lift Current Position", getCurrentPosition());
        t.addData("Lift Power", pwr);
        t.addData("Lift Position Error", tar - getCurrentPosition());
        // t.addData("Lift Current Amps", back.getMotor().motor.);
        t.addData("Lift Reached Target?", reachedTarget());
    }

    public boolean reachedTarget(){
        return pidf.atSetPoint();
    }

    public void resetEncoder(){
        back.resetEncoder();
    }

    public Command SetModeCommand(Mode newMode){
        return new InstantCommand(() -> mode = newMode);
    }

    public Command EngageClutchCommand(){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        this.SetModeCommand(Mode.RAW),
                        new InstantCommand(() -> setManualPower(ENGAGING_POWER))
                ),
                new WaitCommand(ENGAGING_DELAY),
                new ParallelCommandGroup(
                        this.SetModeCommand(Mode.PID),
                        new InstantCommand(this::resetEncoder),
                        new InstantCommand(() -> setTargetPosition(LiftPositions.EXTENDED))
                ),
                new WaitUntilCommand(this::reachedTarget)
        );
    }
}
