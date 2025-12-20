package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.core.util.Artifact;
import org.firstinspires.ftc.teamcode.config.core.util.ArtifactDataSmoother;
import org.firstinspires.ftc.teamcode.config.hardware.CachedMotor;

import kotlin.time.Instant;

@Configurable
public class Intake extends SubsysCore {
    CachedMotor im;
    Servo piv;
    DigitalChannel pin0, pin1; // Purple, Green
    ArtifactDataSmoother smoother;
    double pwr;
    public static double INTAKE_PIVOT_ZERO_OFFSET = 0;
    public static double INTAKE_PIVOT_DOWN_BALL = 0.92;
    public static double INTAKE_PIVOT_DOWN_EMPTY = 0.96;
    public static double INTAKE_PIVOT_TRANSFER = 0.69;
    public static long TRANSFER_ACTUATION_TIME_MS = 380;
    boolean pivotUp = false;

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
        piv = h.get(Servo.class, "intakePivot");
        pin0 = h.get(DigitalChannel.class, "digital0");
        pin1 = h.get(DigitalChannel.class, "digital1");
        pwr = 0;
        smoother = new ArtifactDataSmoother(50);
        setDefaultCommand(setPowerCommand(Intake.IntakeMotorPowerConfig.STOP));
    }

    public Command setPowerInstant(double newPower){
        return new InstantCommand(() -> pwr = newPower);
    }
    public Command setPowerCommand(double newPower) {
        return new RunCommand(() -> pwr = newPower, this);
    }
    public Command runUntilArtifactSensed(){
        return this.setPowerCommand(IntakeMotorPowerConfig.INTAKE).raceWith(new WaitUntilCommand(() -> smoother.getStableColor() != Artifact.NONE));
    }
    public Command runWithTimeout(int timeout){
        return new ParallelRaceGroup(
                this.setPowerCommand(IntakeMotorPowerConfig.INTAKE),
                new WaitUntilCommand(() -> smoother.getStableColor() != Artifact.NONE),
                new WaitCommand(timeout)
        );
    }
    public Command resetSmootherCommand(){
        return new InstantCommand(() -> smoother.reset());
    }
    public Artifact getCurrentArtifact(){
        return smoother.getStableColor();
    }

    public Command setUpCommand(boolean state){
        return new ConditionalCommand(
                new WaitCommand(TRANSFER_ACTUATION_TIME_MS),
                new InstantCommand(),
                () -> {
                    boolean changed = (pivotUp != state);
                    pivotUp = state;
                    return changed;
                }
        );
    }

    @Override
    public void periodic() {
        im.setPower(pwr);
        piv.setPosition(INTAKE_PIVOT_ZERO_OFFSET + (pivotUp?INTAKE_PIVOT_TRANSFER: (pwr == IntakeMotorPowerConfig.STOP ? INTAKE_PIVOT_DOWN_BALL : INTAKE_PIVOT_DOWN_EMPTY)));
        t.addData("PivotUp", pivotUp);
        t.addData("Intake Power", pwr);
        t.addData("Intake Current", im.getCurrent());
        smoother.addReading(pin0.getState(), pin1.getState());
        t.addData("Intake Artifact", smoother.getStableColor());
    }
}
