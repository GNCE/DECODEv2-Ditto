package org.firstinspires.ftc.teamcode.config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
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

import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.core.util.Artifact;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

@Configurable
public class Spindex extends SubsysCore {
    Servo sp1;

    AnalogInput spos;
    public static int idx;
    public static Artifact[] st = new Artifact[3];
    public static double MIN_POSITION_TOLERANCE = 2;
    public static double ZERO_OFFSET = 10;
    public static double GEAR_RATIO = (double) 2;

    public Spindex(){
        sp1 = h.get(Servo.class, "spin1");
        spos = h.get(AnalogInput.class, "spos");
        sp1.setDirection(Servo.Direction.FORWARD);
        if(Arrays.stream(st).allMatch(Objects::isNull)) emptyStorage();
    }

    public int getCloserIndexToEnd(){
        if(Math.abs(idx) < Math.abs(idx-4)) return 0;
        else return 4;
    }

    public void emptyStorage(){
        st = new Artifact[]{Artifact.NONE, Artifact.NONE, Artifact.NONE};
        idx=0;
    }

    @Override
    public void periodic() {
        if(isFull()) idx = 2;
        else if(isEmpty()) idx=getCloserIndexToEnd();

        sp1.setPosition(getTargetServoPosition());
        t.addData("Storage", Arrays.stream(st).map(Artifact::name).collect(Collectors.joining(", ")));
        t.addData("Current Index", idx);
        t.addData("Selected Artifact", st[idx%3].name());
    }

    public double getTarget(){
        return idx*120+ZERO_OFFSET;
    }
    public double getCurrent(){
        return spos.getVoltage()/3.3*360*GEAR_RATIO;
    }
    public double getTargetServoPosition(){
        return getTarget()/(355*GEAR_RATIO);
    }

    public boolean reachedTarget(){
        return Math.abs(getTarget() - getCurrent()) < MIN_POSITION_TOLERANCE;
    }


    public boolean isFull(){
        return !contains(Artifact.NONE);
    }

    public boolean isEmpty(){
        return Arrays.stream(st).allMatch(art -> art == Artifact.NONE);
    }

    public boolean contains(Artifact tar){
        return Arrays.asList(st).contains(tar);
    }

    public void overrideItem(int slot, Artifact art){
        st[slot] = art;
    }

    public void insertItem(Artifact art){
        if(st[idx%3] != Artifact.NONE) throw new IllegalAccessError("An artifact cannot be inserted into an occupied slot.");
        st[idx%3] = art;
    }
    public void removeItem(){
        st[idx%3] = Artifact.NONE;
    }

    private void setIdx(int newIdx){ idx = newIdx; }

    public Command goToSlot(int idx){
        return new SequentialCommandGroup(
                new InstantCommand(() -> setIdx(idx), this),
                new WaitUntilCommand(this::reachedTarget)
        );
    }

    public int getClosestIndex(Artifact target) {
        int bestIndex = idx;
        int bestDistance = Integer.MAX_VALUE;

        for (int i = 0; i < 5; i++) {
            Artifact slotArtifact = st[i % 3];
            if (slotArtifact == target) {
                int distance = Math.abs(i - idx);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestIndex = i;
                }
            }
        }

        return bestIndex;
    }

    public Command goToSlot(Artifact tar){
        return new DeferredCommand(
                () -> goToSlot(getClosestIndex(tar)),
                Collections.singletonList(this)
        );
    }
}
