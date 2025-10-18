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
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Lift;
import org.firstinspires.ftc.teamcode.config.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public class IntakeRobot extends Robot {
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
    ToggleButtonReader intakeButton;
    IntakeUntilFullCommand intakeUntilFullCommand;

    public IntakeRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2){
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
        this.allianceSelectionButton = new ToggleButtonReader(this.g1, GamepadKeys.Button.CIRCLE);
        this.intakeButton = new ToggleButtonReader(this.g1, GamepadKeys.Button.SQUARE);

        register(intake, door, spindex);

        intakeUntilFullCommand = new IntakeUntilFullCommand(intake, door, spindex);

        this.lt = new LoopTimer();
    }

    public void allianceSelection(){
        t.addData("Current Alliance", isRed?"RED": "BLUE");
        isRed = !allianceSelectionButton.getState();

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
        if(intakeButton.stateJustChanged()){
            if(intakeButton.getState()) intakeUntilFullCommand.schedule();
            else intakeUntilFullCommand.cancel();
        }
    }
    public void endPeriodic(){
        this.run();
        lt.end();
        t.addData("Loop Time (ms)", lt.getMs());
        t.addData("Loop Frequency (Hz)", lt.getHz());
        t.update();
    }
}
