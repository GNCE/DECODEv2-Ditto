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
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.ToggleButtonReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.commands.IntakeUntilFullSafeCommand;
import org.firstinspires.ftc.teamcode.config.commands.OuttakeAllCommand;
import org.firstinspires.ftc.teamcode.config.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.config.commands.TransferAllCommand;
import org.firstinspires.ftc.teamcode.config.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.config.core.util.Artifact;
import org.firstinspires.ftc.teamcode.config.core.util.ArtifactMatch;
import org.firstinspires.ftc.teamcode.config.core.util.Motif;
import org.firstinspires.ftc.teamcode.config.core.util.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.SubsystemConfig;
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
    Pose goalPose;
    OpModeType opModeType;

    public static boolean isRed;
    public static double TURRET_OFFSET_M = 0.08288647; // 82.88647 mm
    ToggleButtonReader allianceSelectionButton;
    ToggleButton intakeButton;
    IntakeUntilFullSafeCommand intakeUntilFullSafeCommand;
    List<SubsystemConfig> subsysList;
    boolean [] enabledSubsys = new boolean[SubsystemConfig.values().length];
    int slotSelect = 0;

    public boolean hasSubsystem(SubsystemConfig subsystem){
        return enabledSubsys[subsystem.ordinal()];
    }

    public boolean hasSubsystems(List<SubsystemConfig> subsystems){
        for(SubsystemConfig subsystem: subsystems){
            if(!hasSubsystem(subsystem)) return false;
        }
        return true;
    }

    public MyRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, List<SubsystemConfig> subsysList, Pose startingPose, OpModeType opModeType){
        this.opModeType = opModeType;
        this.subsysList = subsysList;
        for(SubsystemConfig subsystem: subsysList){
            enabledSubsys[subsystem.ordinal()] = true;
        }

        this.h = h;
        this.t = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), t);
        hubs = this.h.getAll(LynxModule.class);
        for(LynxModule hub: hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        this.g1 = new GamepadEx(g1);
        this.g2 = new GamepadEx(g2);

        SubsysCore.setGlobalParameters(this.h, this.t);

        if(hasSubsystem(SubsystemConfig.INTAKE)){
            this.intake = new Intake();
        }
        if(hasSubsystem(SubsystemConfig.FOLLOWER)){
            this.f = Constants.createFollower(this.h);
            this.f.setStartingPose(startingPose);
            this.f.update();
            if(hasSubsystem(SubsystemConfig.LL)){
                this.ll = new Limelight(this.f, true);
            }
            if(hasSubsystem(SubsystemConfig.TURRET)) turret = new Turret(endTurretWrapCount);
        }
        if(hasSubsystem(SubsystemConfig.SHOOTER)) this.shooter = new Shooter();
        if(hasSubsystem(SubsystemConfig.DOOR)) this.door = new Door();
        if(hasSubsystem(SubsystemConfig.LIFT)) this.lift = new Lift();
        if(hasSubsystem(SubsystemConfig.SPINDEX)) this.spindex = new Spindex();

        if(this.opModeType == OpModeType.TELEOP){
            if(hasSubsystems(Arrays.asList(SubsystemConfig.INTAKE, SubsystemConfig.DOOR, SubsystemConfig.SPINDEX))){
                this.intakeButton = new ToggleButton(false);
                this.intakeUntilFullSafeCommand = new IntakeUntilFullSafeCommand(intake, door, spindex);
            }
        }

        this.allianceSelectionButton = new ToggleButtonReader(this.g1, GamepadKeys.Button.DPAD_UP);
        this.lt = new LoopTimer();
    }

    public MyRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, List<SubsystemConfig> subsysList){
        this(h, t, g1, g2, subsysList, autoEndPose == null ? new Pose(54.69, 6.74, Math.toRadians(180)) : autoEndPose, OpModeType.TELEOP);
    }

    public MyRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2){
        this(h, t, g1, g2, Arrays.asList(SubsystemConfig.values()));
    }

    public MyRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, Pose startingPose){
        this(h, t, g1, g2, Arrays.asList(SubsystemConfig.values()), startingPose, OpModeType.AUTO);
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

    public void startDrive(){
        f.startTeleopDrive();
        f.update();
    }

    public void driveControls(){
        if(g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.8) f.setTeleOpDrive(g1.getLeftY()*0.3, -g1.getLeftX()*0.3, -g1.getRightX()*0.3, true);
        else f.setTeleOpDrive(g1.getLeftY(), -g1.getLeftX(), -g1.getRightX(), true);
    }

    private final Pose blueGoalPose = new Pose(11.5, 138);

    public void startInitLoop(){
        lt.start();
        resetCache();
        allianceSelection();
        allianceSelectionButton.readValue();
        g1.readButtons();
        g2.readButtons();
        if(!isRed) goalPose = blueGoalPose;
        else goalPose = blueGoalPose.mirror();
    }

    public void endInitLoop(){
        if(opModeType == OpModeType.AUTO) this.run(); // TODO: Need to make sure that they are all paused, especially shooter.
        if(hasSubsystem(SubsystemConfig.FOLLOWER)) f.update();
        lt.end();
        t.addData("Loop Time (ms)", lt.getMs());
        t.addData("Loop Frequency (Hz)", lt.getHz());
        t.update();
    }

    public void stop(){
        if(hasSubsystem(SubsystemConfig.FOLLOWER)) autoEndPose = f.getPose();
        if(hasSubsystem(SubsystemConfig.TURRET)) endTurretWrapCount = turret.getWrapCount();
    }

    public void resetCache(){
        for(LynxModule hub: hubs){
            hub.clearBulkCache();
        }
    }

    public void startPeriodic(){
        lt.start();
        resetCache();
        if(hasSubsystem(SubsystemConfig.FOLLOWER)){
            Pose turretPose = new Pose(
                    f.getPose().getX() + TURRET_OFFSET_M* Math.cos(f.getPose().getHeading()),
                    f.getPose().getY() + TURRET_OFFSET_M* Math.sin(f.getPose().getHeading()),
                    f.getPose().getHeading());
            if(hasSubsystem(SubsystemConfig.SHOOTER)) shooter.input(turretPose, goalPose);
            if(hasSubsystem(SubsystemConfig.TURRET)) turret.input(turretPose, goalPose);
        }
    }
    public void runIntakeTeleop(){
        if(intakeUntilFullSafeCommand.isFinished()) intakeButton.setVal(false);
        if(intakeButton.input(g1.getButton(GamepadKeys.Button.SQUARE))){
            if(intakeButton.getVal()) intakeUntilFullSafeCommand.schedule();
            else intakeUntilFullSafeCommand.cancel();
        }

        t.addData("Intake Active", intakeButton.getVal());
        t.addData("Intake Scheduled", intakeUntilFullSafeCommand.isScheduled());
        t.addData("IntakeCommand finished?", intakeUntilFullSafeCommand.isFinished());
        t.addData("Spindex full?", spindex.isFull());
    }
    public void runTransferTeleop(){
        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)
                && spindex.contains(Artifact.PURPLE))
            schedule(new TransferCommand(ArtifactMatch.PURPLE, spindex, door, intake));
        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)
                && spindex.contains(Artifact.GREEN))
            schedule(new TransferCommand(ArtifactMatch.GREEN, spindex, door, intake));
        if (g1.wasJustPressed(GamepadKeys.Button.TRIANGLE) && !spindex.isEmpty())
            schedule(new TransferAllCommand(intake, spindex, door));
    }

    public void runShootTeleop(){
        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)
                && spindex.contains(Artifact.PURPLE))
            schedule(new OuttakeCommand(ArtifactMatch.PURPLE, intake, spindex, turret, shooter, door));
        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)
                && spindex.contains(Artifact.GREEN))
            schedule(new OuttakeCommand(ArtifactMatch.GREEN, intake, spindex, turret, shooter, door));
        if (g1.wasJustPressed(GamepadKeys.Button.TRIANGLE) && !spindex.isEmpty())
            schedule(new OuttakeAllCommand(intake, spindex, turret, shooter, door));
    }
    public void endPeriodic(){
        this.run();
        if(hasSubsystem(SubsystemConfig.FOLLOWER)) {
            f.update();
            t.addData("Current Pose", f.getPose());
            t.addData("Current Velocity", f.getVelocity());
        }
        g1.readButtons();
        g2.readButtons();
        lt.end();
        t.addData("Loop Time (ms)", lt.getMs());
        t.addData("Loop Frequency (Hz)", lt.getHz());
        t.update();
    }
}
