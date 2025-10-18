package org.firstinspires.ftc.teamcode.config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.core.util.Artifact;

import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

@Configurable
public class Spindex extends SubsysCore {
    CRServo sp1, sp2;

    AnalogInput spos;
    double ca;
    public static int idx;
    public static Artifact[] st = new Artifact[3];
    PIDController spid;
    public static double kp = 0.0025, ki = 0.0005, kd = 0; // TODO: Edit
    public static double MIN_POSITION_TOLERANCE = 2; //error limit or deadzone range, e.g 5 would have 10 degrees of variace
    public static double ZERO_OFFSET = 107;
    public static double GEAR_RATIO = (double) 2;
    public static double MIN_POWER = 0.05;

    public Spindex(){
        sp1 = h.get(CRServo.class, "spin1");
        sp2 = h.get(CRServo.class, "spin2");

        spos = h.get(AnalogInput.class, "spos");

        sp1.setDirection(DcMotorSimple.Direction.FORWARD);
        sp2.setDirection(DcMotorSimple.Direction.REVERSE);
        spid = new PIDController(kp, ki, kd);
        emptyStorage();
    }

    public void emptyStorage(){
        st = new Artifact[]{Artifact.NONE, Artifact.NONE, Artifact.NONE};
        idx=0;
    }

    @Override
    public void periodic() {
        spid.setPID(kp, ki, kd);

        ca = spos.getVoltage()/3.3*360;
        double cur = (ca*GEAR_RATIO)%360;
        double tar = (idx*120+ZERO_OFFSET)%360;
        double err =  tar-cur;
        if(err > 180) err = err-360;
        else if(err < -180) err = err+360;

        double pwr = spid.calculate(err, 0);
        if(pwr < MIN_POWER && pwr > 0) pwr = MIN_POWER;
        else if(pwr > -MIN_POWER && pwr < 0) pwr = -MIN_POWER;

        if (Math.abs(err) > MIN_POSITION_TOLERANCE) {
            sp1.setPower(pwr);
            sp2.setPower(pwr);
        } else {
            sp1.setPower(0);
            sp2.setPower(0);
        }

        t.addData("Storage", Arrays.stream(st).map(Artifact::name).collect(Collectors.joining(", ")));
        t.addData("Current Index", idx);
        t.addData("Selected Artifact", st[idx].name());
        t.addData("SpindexerError", err);
        t.addData("SpindexerPosition", cur);
        t.addData("SpindexerPwr", pwr);
        t.addLine("pwr:"+pwr);
    }

    public boolean reachedTarget(){
        return Math.abs(idx*120 - ca) < 8;
    }

    public boolean contains(Artifact tar){
        return Arrays.asList(st).contains(tar);
    }

    public void insertItem(Artifact art){
        if(st[idx] != Artifact.NONE) throw new IllegalAccessError("An artifact cannot be inserted into an occupied slot.");
        st[idx] = art;
    }
    public void removeItem(){
        st[idx] = Artifact.NONE;
    }

    public Command goToSlot(Artifact tar){
        if(st[idx] == tar) return new InstantCommand();
        int cw = (idx+1)%3, ccw = (idx+2)%3;
        if(st[cw] == tar || st[ccw] == tar){
            if(st[cw]==tar) idx = cw;
            else idx = ccw;
            return new WaitUntilCommand(this::reachedTarget);
        }
        throw new IllegalAccessError("The Spindexer storage does not have the requested Artifact. Use contains() to check before calling the command.");
    }
}
