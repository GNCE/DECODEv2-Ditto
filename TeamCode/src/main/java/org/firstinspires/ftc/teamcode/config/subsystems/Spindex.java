package org.firstinspires.ftc.teamcode.config.subsystems;

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

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.core.util.Artifact;

import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

public class Spindex extends SubsysCore {
    CRServo sp1, sp2;
    AnalogInput spos;
    double ca;
    public static int idx;
    public static Artifact[] st = new Artifact[3];
    PIDController spid;
    public static double kp = 0.5, ki = 0, kd = 0.05; // TODO: Edit

    public Spindex(){
        sp1 = h.get(CRServo.class, "spin1");
        sp2 = h.get(CRServo.class, "spin2");
        spos = h.get(AnalogInput.class, "spos");
        sp1.setDirection(DcMotorSimple.Direction.FORWARD);
        sp2.setDirection(DcMotorSimple.Direction.REVERSE);
        spid = new PIDController(kp, ki, kd);
    }

    public void emptyStorage(){
        st = new Artifact[]{Artifact.NONE, Artifact.NONE, Artifact.NONE};
        idx=0;
    }

    @Override
    public void periodic() {
        ca = spos.getVoltage()/3.3*360;
        spid.setPID(kp, ki, kd);
        double err = idx*120 - ca;
        if(err > 180) err = err-360;
        else if(err < -180) err = err+360;
        double pwr = spid.calculate(0, err);
        sp1.setPower(pwr);
        sp2.setPower(pwr);

        t.addData("Storage", Arrays.stream(st).map(Artifact::name).collect(Collectors.joining(", ")));
        t.addData("Current Index", idx);
        t.addData("Selected Artifact", st[idx].name());
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
