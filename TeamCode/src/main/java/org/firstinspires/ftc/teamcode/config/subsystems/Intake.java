package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.hardware.CachedMotor;

@Configurable
public class Intake extends SubsysCore {
    CachedMotor im;
    double pwr;

    public enum IntakeMotorPowerStates {
        INTAKE, TRANSFER, REJECT, STOP
    }
    public static class IntakeMotorPowerConfig {
        public static double INTAKE = 1;
        public static double TRANSFER = 0.6;
        public static double STOP = 0;
        public static double REJECT = -0.85;
    }

    public Intake(){
        im = new CachedMotor(h.get(DcMotorEx.class, "intakeMotor"));
        im.setDirection(DcMotorSimple.Direction.REVERSE);
        pwr = 0;
        setDefaultCommand(setPowerCommand(Intake.IntakeMotorPowerConfig.STOP));
    }

    public Command setPowerInstant(double newPower){
        return new InstantCommand(() -> pwr = newPower);
    }
    public Command setPowerCommand(double newPower) {
        return new RunCommand(() -> pwr = newPower, this);
    }

    public Command runWithTimeout(int timeout){
        return new ParallelRaceGroup(
                this.setPowerCommand(IntakeMotorPowerConfig.INTAKE),
                new WaitCommand(timeout)
        );
    }

    @Override
    public void periodic() {
        im.setPower(pwr);
        t.addData("Intake Power", pwr);
        t.addData("Intake Current", im.getCurrent());
    }
}
