package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.hardware.CachedMotor;

@Configurable
public class Intake extends SubsysCore {
    CachedMotor im, tr;
    ServoEx servo;
    double pwr;

    public static double INTAKE_DOWN = 0.0;
    public static double INTAKE_UP = 0.0;
    public static double TRANSFER_I_POWER = 0.7;
    public static double TRANSFER_T_POWER = 0.7;

    public enum SERVO_POSITIONS {
        INTAKE_UP(0), INTAKE_DOWN(0.3);

        public final double pos;

        SERVO_POSITIONS(double pos) {
            this.pos = pos;
        }
    }

    public Intake(){
        im = new CachedMotor(h.get(DcMotorEx.class, "intakeMotor"));
        im.setDirection(DcMotorSimple.Direction.REVERSE);

        tr = new CachedMotor(h.get(DcMotorEx.class, "transferMotor"));
        tr.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = new ServoEx(h, "intakePitch");
        pwr = 0;
        mode = Mode.INTAKE;
        setDefaultCommand(new RunCommand(() -> mode = Mode.INTAKE, this));
    }

    public void setIntakePitch(SERVO_POSITIONS sp){
        servo.set(sp.pos);
    }

    public double getCurrent(){
        return im.getCurrent();
    }

    public enum Mode {
        INTAKE,
        TRANSFER,
        DISABLE,
        REVERSE
    }

    Mode mode;

    int size;

    public void inputStorageSize(int size){
        this.size = size;
    }
    public void setMode(Mode mode){
        this.mode = mode;
    }

    @Override
    public void periodic() {
        switch (mode) {
            case DISABLE:
                im.setPower(0);
                tr.setPower(0);
                break;
            case REVERSE:
                im.setPower(-1);
                tr.setPower(-1);
                break;
            case INTAKE:
                if (size <= 0) tr.setPower(1);
                else tr.setPower(0);
                if (size >= 3){
                    im.setPower(0);
                    setIntakePitch(SERVO_POSITIONS.INTAKE_UP);
                } else {
                    im.setPower(1);
                    setIntakePitch(SERVO_POSITIONS.INTAKE_DOWN);
                }
                break;
            case TRANSFER:
                im.setPower(TRANSFER_I_POWER);
                tr.setPower(TRANSFER_T_POWER);
                break;
        }
        if(mode != Mode.INTAKE) setIntakePitch(SERVO_POSITIONS.INTAKE_UP);
        t.addData("Intake Mode", mode.name());
    }
}
