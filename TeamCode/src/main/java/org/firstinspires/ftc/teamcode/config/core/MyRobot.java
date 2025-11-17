package org.firstinspires.ftc.teamcode.config.core;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.Robot;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.ToggleButtonReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.commands.IntakeUntilFullCommand;
import org.firstinspires.ftc.teamcode.config.commands.TransferAllCommand;
import org.firstinspires.ftc.teamcode.config.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.config.core.util.Artifact;
import org.firstinspires.ftc.teamcode.config.core.util.ArtifactMatch;
import org.firstinspires.ftc.teamcode.config.core.util.Motif;
import org.firstinspires.ftc.teamcode.config.core.util.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.ToggleButton;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Lift;
import org.firstinspires.ftc.teamcode.config.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class MyRobot extends Robot {
    HardwareMap h;
    JoinedTelemetry t;
    public Follower f;
    List<LynxModule> hubs;
    LoopTimer lt;
    public Intake intake;
    public Turret turret;
    public Spindex spindex;
    public Door door;
    public Lift lift;
    public Limelight ll;
    public Shooter shooter;
    public GamepadEx g1, g2;
    public static Pose autoEndPose;
    public static Motif currentMotif;
    public static int endTurretWrapCount;
    OpModeType opModeType;

    public static boolean isRed = true;
    ToggleButtonReader allianceSelectionButton;
    ToggleButton intakeButton;
    IntakeUntilFullCommand intakeUntilFullCommand;


    int slotSelect = 0;

    public MyRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2){
        this.h = h;
        this.t = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), t);
        hubs = this.h.getAll(LynxModule.class);
        for(LynxModule hub: hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        this.f = Constants.createFollower(this.h);
        this.g1 = new GamepadEx(g1);
        this.g2 = new GamepadEx(g2);

        SubsysCore.setGlobalParameters(this.h, this.t);
        this.intake = new Intake();
        this.ll = new Limelight(this.f, true);

        this.turret = new Turret(this.f, this.ll, endTurretWrapCount, true);
        this.shooter = new Shooter();
        this.door = new Door();
        this.lift = new Lift();
        this.spindex = new Spindex();
        this.allianceSelectionButton = new ToggleButtonReader(this.g1, GamepadKeys.Button.DPAD_UP);

        this.lt = new LoopTimer();
    }

    public MyRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, OpModeType opModeType){
        this(h, t, g1, g2);
        this.opModeType = opModeType;
        this.intakeButton = new ToggleButton(false);
        this.intakeUntilFullCommand = new IntakeUntilFullCommand(intake, door, spindex);
        this.f.setStartingPose(autoEndPose == null ? new Pose(0, 0, 0) : autoEndPose);
    }

    public MyRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, OpModeType opModeType, Pose startingPose){
        this(h, t, g1, g2);
        this.opModeType = opModeType;
        this.f.setStartingPose(startingPose);
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

    public void startInitLoop(){
        lt.start();
        resetCache();
        allianceSelection();
        preloadSelection();
        allianceSelectionButton.readValue();
        g1.readButtons();
        g2.readButtons();
    }

    public void endInitLoop(){
        lt.end();
        t.addData("Loop Time (ms)", lt.getMs());
        t.addData("Loop Frequency (Hz)", lt.getHz());
        t.update();
    }

    public void stop(){
        autoEndPose = f.getPose();
        endTurretWrapCount = turret.getWrapCount();
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

        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)
                && spindex.contains(Artifact.PURPLE))
            schedule(new TransferCommand(ArtifactMatch.PURPLE, spindex, door, intake));
        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)
                && spindex.contains(Artifact.GREEN))
            schedule(new TransferCommand(ArtifactMatch.GREEN, spindex, door, intake));
        if (g1.wasJustPressed(GamepadKeys.Button.TRIANGLE) && !spindex.isEmpty())
            schedule(new TransferAllCommand(intake, spindex, door));

        t.addData("Intake Active", intakeButton.getVal());
        t.addData("Intake Scheduled", intakeUntilFullCommand.isScheduled());
        t.addData("IntakeCommand finished?", intakeUntilFullCommand.isFinished());
        t.addData("Spindex full?", spindex.isFull());
    }
    public void endPeriodic(){
        this.run();
        f.update();
        g1.readButtons();
        g2.readButtons();
        lt.end();
        t.addData("Loop Time (ms)", lt.getMs());
        t.addData("Loop Frequency (Hz)", lt.getHz());
        t.update();
    }
}
