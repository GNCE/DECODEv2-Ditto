package org.firstinspires.ftc.teamcode.config.core;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.Robot;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.ToggleButtonReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.commands.IntakeUntilFullCommand;
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
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

public class MyRobot extends Robot {
    HardwareMap h;
    public JoinedTelemetry t;
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

    public static Boolean isRed = null;
    public static double TURRET_OFFSET_INCHES = 3.26322835;
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

    public MyRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, List<SubsystemConfig> subsysList, OpModeType opModeType){
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
        if(hasSubsystem(SubsystemConfig.LL)){
            this.ll = new Limelight();
        }
        if(hasSubsystem(SubsystemConfig.FOLLOWER)){
            this.f = Constants.createFollower(this.h);

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

        if(isRed == null) isRed = false;
        this.lt = new LoopTimer();
    }
    public MyRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, List<SubsystemConfig> subsysList){
        this(h, t, g1, g2, subsysList, OpModeType.TELEOP);
    }

    public MyRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, OpModeType opModeType){
        this(h, t, g1, g2, Arrays.asList(SubsystemConfig.values()), opModeType);
    }

    public void allianceSelection(){
        if(g1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) isRed = !isRed;
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

    private final Pose blueGoalPose = new Pose(16.534, 131.89); // 141x141 x:1.5-142.5 y:0-141

    public void startInitLoop(){
        lt.start();
        resetCache();
        allianceSelection();
        g1.readButtons();
        g2.readButtons();
        if(!isRed) goalPose = blueGoalPose;
        else goalPose = blueGoalPose.mirror();
    }

    public void endInitLoop(){
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
                    f.getPose().getX() + TURRET_OFFSET_INCHES * Math.cos(f.getPose().getHeading()),
                    f.getPose().getY() + TURRET_OFFSET_INCHES * Math.sin(f.getPose().getHeading()),
                    f.getPose().getHeading());
            if(hasSubsystem(SubsystemConfig.SHOOTER)) shooter.input(turretPose, goalPose);
            if(hasSubsystem(SubsystemConfig.TURRET)) turret.input(turretPose, goalPose);
        }
    }
    public void runIntakeTeleop(){
        if(!intakeUntilFullSafeCommand.isScheduled()) intakeButton.setVal(false);
        if(intakeButton.input(g1.getButton(GamepadKeys.Button.SQUARE))){
            if(intakeButton.getVal()) intakeUntilFullSafeCommand.schedule();
            else intakeUntilFullSafeCommand.cancel();
        }

        if(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9) schedule(intake.setPowerInstant(-1));

        if(g1.wasJustPressed(GamepadKeys.Button.CIRCLE)) intake.smoother.addReading(true, false);
        else if(g1.wasJustPressed(GamepadKeys.Button.CROSS)) intake.smoother.addReading(true, true);

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
        double lt = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rt = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if(lt > 0.1 || rt > 0.1) Turret.MANUAL_OFFSET += (rt - lt)/2;

        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)
                && spindex.contains(Artifact.PURPLE))
            schedule(new OuttakeCommand(ArtifactMatch.PURPLE, intake, spindex, turret, shooter, door));
        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)
                && spindex.contains(Artifact.GREEN))
            schedule(new OuttakeCommand(ArtifactMatch.GREEN, intake, spindex, turret, shooter, door));
        if (g1.wasJustPressed(GamepadKeys.Button.TRIANGLE) && !spindex.isEmpty())
            schedule(new OuttakeAllCommand(intake, spindex, turret, shooter, door));
    }

    public void liftTeleop(){
        if(g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) schedule(new InstantCommand(()->lift.setTargetPosition(Lift.LiftPositions.EXTENDED)));
    }

    public void endPeriodic() {
        this.run();
        if (hasSubsystem(SubsystemConfig.FOLLOWER)) {
            f.update();
            t.addData("Current Pose", f.getPose());
            t.addData("Current Velocity", f.getVelocity());
            t.addData("Follower Busy?" , f.isBusy());
        }
        if(hasSubsystems(List.of(SubsystemConfig.SHOOTER, SubsystemConfig.TURRET, SubsystemConfig.INTAKE, SubsystemConfig.DOOR, SubsystemConfig.SPINDEX))){
            t.addLine();
            t.addLine("SHOOTER DATA");
            if(!shooter.readyToShoot()) t.addLine("Shooter Not Ready");
            if(!turret.reachedTarget()) t.addLine("Turret Not Ready");
            if(!door.isOpen()) t.addLine("Door Not Open");
            if(!spindex.reachedTarget()) t.addLine("Spindex Not Ready");
            t.addData("Shoot Ready?", shooter.readyToShoot() && turret.reachedTarget() && door.isOpen() && spindex.reachedTarget());
            t.addLine();
        }
        g1.readButtons();
        g2.readButtons();
        lt.end();
        t.addData("Current Motif", currentMotif == null ? "Not Detected" : currentMotif.name());
        t.addData("Loop Time (ms)", lt.getMs());
        t.addData("Loop Frequency (Hz)", lt.getHz());
        t.update();
    }

    public void overrideAutoEndPose(Pose newAutoEndPose){
        autoEndPose = newAutoEndPose;
    }

    public void onStart(){
        if(hasSubsystem(SubsystemConfig.FOLLOWER)){
            if(autoEndPose == null) autoEndPose = new Pose(76.9329, 6.742126, Math.toRadians(180));
            this.f.setStartingPose(autoEndPose);
            this.f.update();
            if(opModeType == OpModeType.TELEOP) this.startDrive();
        }
    }

    /// Commands ///

    public Command shoot(ArtifactMatch artifactMatch){
        return new OuttakeCommand(artifactMatch, this.intake, this.spindex, this.turret, this.shooter, this.door);
    }

    public Command shootMotif(){
        return new SelectCommand(
                new HashMap<Object, Command>(){{
                    put(Motif.GPP, new SequentialCommandGroup(
                            shoot(ArtifactMatch.GREEN),
                            shoot(ArtifactMatch.PURPLE),
                            shoot(ArtifactMatch.PURPLE)
                    ));
                    put(Motif.PGP, new SequentialCommandGroup(
                            shoot(ArtifactMatch.PURPLE),
                            shoot(ArtifactMatch.GREEN),
                            shoot(ArtifactMatch.PURPLE)
                    ));
                    put(Motif.PPG, new SequentialCommandGroup(
                            shoot(ArtifactMatch.PURPLE),
                            shoot(ArtifactMatch.PURPLE),
                            shoot(ArtifactMatch.GREEN)
                    ));
                }},
                () -> MyRobot.currentMotif
        );
    }

    public Command shootAll(){
        return new OuttakeAllCommand(intake, spindex, turret, shooter, door);
    }

    public Command shootMotifSafe(){
            return new ConditionalCommand(
                    shootMotif(),
                    shootAll(),
                    () -> spindex.canMotif()
            );
    }

    public Command goTo(Pose tar){
        return new SequentialCommandGroup(
                new InstantCommand(() -> f.followPath(
                        f.pathBuilder()
                                .addPath(new BezierLine(f.getPose(), tar))
                                .setLinearHeadingInterpolation(f.getHeading(), tar.getHeading())
                                .build()
                )),
                new WaitUntilCommand(() -> !f.isBusy())
        );
    }

    public Command intakeAll(){
        return new IntakeUntilFullCommand(intake, door, spindex);
    }

    public Command intakeForcedWithTimeout(int timeout, Artifact artifact){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        door.setOpenCommand(false),
                        spindex.goToSlot(ArtifactMatch.NONE),
                        intake.setPowerInstant(Intake.IntakeMotorPowerConfig.STOP)
                ),
                intake.resetSmootherCommand(),
                intake.runWithTimeout(timeout),
                intake.setPowerInstant(Intake.IntakeMotorPowerConfig.STOP),
                new InstantCommand(() -> spindex.insertItem(intake.getCurrentArtifact() == Artifact.NONE ? artifact : intake.getCurrentArtifact())),
                intake.resetSmootherCommand()
        );
    }
}
