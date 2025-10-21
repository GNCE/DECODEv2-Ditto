package org.firstinspires.ftc.teamcode.config.core;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.ToggleButtonReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.commands.IntakeUntilFullCommand;
import org.firstinspires.ftc.teamcode.config.core.util.Artifact;
import org.firstinspires.ftc.teamcode.config.core.util.ToggleButton;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Lift;
import org.firstinspires.ftc.teamcode.config.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class IntakeOuttakeRobot extends Robot {
    HardwareMap h;
    JoinedTelemetry t;
    Follower f;
    List<LynxModule> hubs;
    LoopTimer lt;
    Intake intake;
    Turret turret;
    Spindex spindex;
    Door door;
    Lift lift;
    Limelight ll;
    Shooter shooter;
    GamepadEx g1, g2;
    public static Pose autoEndPose;

    public static boolean isRed = true;
    ToggleButtonReader allianceSelectionButton;
    ToggleButton intakeButton;
    IntakeUntilFullCommand intakeUntilFullCommand;

    int slotSelect = 0;

    public IntakeOuttakeRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2){
        this.h = h;
        this.t = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), t);
        hubs = this.h.getAll(LynxModule.class);
        for(LynxModule hub: hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        this.g1 = new GamepadEx(g1);
        this.g2 = new GamepadEx(g2);

        SubsysCore.setGlobalParameters(this.h, this.t);
        this.intake = new Intake();
        this.door = new Door();
        this.spindex = new Spindex();
        this.allianceSelectionButton = new ToggleButtonReader(this.g1, GamepadKeys.Button.DPAD_UP);
        this.intakeButton = new ToggleButton(false);

        register(intake, door, spindex);

        intakeUntilFullCommand = new IntakeUntilFullCommand(intake, door, spindex);

        this.lt = new LoopTimer();
    }

    public void allianceSelection(){
        isRed = !allianceSelectionButton.getState();
        t.addData("Current Alliance", isRed?"RED": "BLUE");
    }


    public void preloadSelection(){
        if(g1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) slotSelect = Math.max(0, slotSelect-1);
        if(g1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) slotSelect = Math.min(2, slotSelect+1);

        if(g1.wasJustPressed(GamepadKeys.Button.CIRCLE)) spindex.overrideItem(slotSelect, Artifact.NONE);
        if(g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) spindex.overrideItem(slotSelect, Artifact.PURPLE);
        if(g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) spindex.overrideItem(slotSelect, Artifact.GREEN);

        t.addData("Storage", Arrays.stream(Spindex.st).map(Artifact::name).collect(Collectors.joining(", ")));
        t.addData("Current Index", slotSelect);
        t.addData("Selected Artifact", Spindex.st[slotSelect].name());
    }

    public void init_loop(){
        lt.start();
        resetCache();
        allianceSelection();
        preloadSelection();
        allianceSelectionButton.readValue();
        g1.readButtons();
        g2.readButtons();
        lt.end();
        t.addData("Loop Time (ms)", lt.getMs());
        t.addData("Loop Frequency (Hz)", lt.getHz());
        t.update();
    }

    public void stop(){

    }

    public void resetCache(){
        for(LynxModule hub: hubs){
            hub.clearBulkCache();
        }
    }

    public void startPeriodic(){
        lt.start();
        resetCache();
    }
    public void runIntakeTeleop(){
        if(intakeUntilFullCommand.isFinished()) intakeButton.setVal(false);
        if(intakeButton.input(g1.getButton(GamepadKeys.Button.SQUARE))){
            if(intakeButton.getVal()) intakeUntilFullCommand.schedule();
            else intakeUntilFullCommand.cancel();
        }

        t.addData("Intake Active", intakeButton.getVal());
        t.addData("Intake Scheduled", intakeUntilFullCommand.isScheduled());
        t.addData("IntakeCommand finished?", intakeUntilFullCommand.isFinished());
        t.addData("Spindex full?", spindex.isFull());
    }
    public void endPeriodic(){
        this.run();
        g1.readButtons();
        g2.readButtons();
        lt.end();
        t.addData("Loop Time (ms)", lt.getMs());
        t.addData("Loop Frequency (Hz)", lt.getHz());
        t.update();
    }
}
