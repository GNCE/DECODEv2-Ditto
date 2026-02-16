package org.firstinspires.ftc.teamcode.config.core;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
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
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.ToggleButtonReader;
import com.seattlesolvers.solverslib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.commands.LiftCommand;
import org.firstinspires.ftc.teamcode.config.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.config.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.config.core.util.Artifact;
import org.firstinspires.ftc.teamcode.config.core.util.ArtifactMatch;
import org.firstinspires.ftc.teamcode.config.core.util.Motif;
import org.firstinspires.ftc.teamcode.config.core.util.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.PoseEKF;
import org.firstinspires.ftc.teamcode.config.core.util.SAT2D;
import org.firstinspires.ftc.teamcode.config.core.util.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.core.util.ToggleButton;
import org.firstinspires.ftc.teamcode.config.core.util.VisionMeasurement;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Lift;
import org.firstinspires.ftc.teamcode.config.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Storage;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class MyRobot extends Robot {
    HardwareMap h;
    public JoinedTelemetry t;
    public Follower f;
    List<LynxModule> hubs;
    LoopTimer lt;
    public Intake intake;
    public Turret turret;
    public Door door;
    public Storage storage;
    public Lift lift;
    public Limelight ll;
    public Shooter shooter;
    public PoseEKF ekf;
    Timer runtime;
    public GamepadEx g1, g2;
    public static Pose autoEndPose;
    public static Motif currentMotif;
    public static int endTurretWrapCount;
    Pose goalPose;
    OpModeType opModeType;

    public static Boolean isRed = null;
    public static double TURRET_OFFSET_INCHES = -0.905511811;

    public static double chassisLeft = 5.5196850394; // Inches
    public static double chassisRight = 5.5196850394;
    public static double chassisBack = 6.8661417323;
    public static double chassisFront = 10.413385827;

    public static double chassisLeftOut = 5.79;
    public static double chassisRightOut = 5.5196850394;

    SAT2D.PolygonSet launchZones = new SAT2D.PolygonSet()
            .add("Close",
                    new SAT2D.Point2(0, 144),
                    new SAT2D.Point2(144, 144),
                    new SAT2D.Point2(72, 72))
            .add("Far",
                    new SAT2D.Point2(72, 24),
                    new SAT2D.Point2(48, 0),
                    new SAT2D.Point2(96, 0));

    ToggleButton autoFireButton, turretAlwaysReadyButton;
    OuttakeCommand outtakeCommand;
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
            this.storage = new Storage();
        }
        if(hasSubsystem(SubsystemConfig.LL)){
            this.ll = new Limelight();
        }
        if(hasSubsystem(SubsystemConfig.FOLLOWER)){
            this.f = Constants.createFollower(this.h);

            if(hasSubsystem(SubsystemConfig.TURRET)){
                turret = new Turret(endTurretWrapCount);
                if(this.opModeType == OpModeType.TELEOP) turretAlwaysReadyButton = new ToggleButton(false);
            }
        }
        if(hasSubsystem(SubsystemConfig.SHOOTER)) this.shooter = new Shooter();
        if(hasSubsystem(SubsystemConfig.DOOR)) this.door = new Door();
        if(hasSubsystem(SubsystemConfig.LIFT)) this.lift = new Lift();

        if(this.opModeType == OpModeType.TELEOP){
            if(hasSubsystems(Arrays.asList(SubsystemConfig.INTAKE, SubsystemConfig.DOOR, SubsystemConfig.SHOOTER, SubsystemConfig.TURRET))){
                this.autoFireButton = new ToggleButton(false);
            }
        }

        if(hasSubsystems(List.of(SubsystemConfig.INTAKE, SubsystemConfig.DOOR, SubsystemConfig.SHOOTER, SubsystemConfig.TURRET))){
            outtakeCommand = new OuttakeCommand(intake, turret, shooter, door, storage);
        }

        if(isRed == null) isRed = false;
        this.lt = new LoopTimer();
        this.runtime = new Timer();
        this.ekf = new PoseEKF();
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
    }

    public void startDrive(){
        f.startTeleopDrive();
        f.update();
        runtime.resetTimer();
    }

    public void driveControls(){
        if(g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.8) f.setTeleOpDrive(g1.getLeftY()*0.3, -g1.getLeftX()*0.3, -g1.getRightX()*0.3, true);
        else f.setTeleOpDrive(g1.getLeftY(), -g1.getLeftX(), -g1.getRightX(), true);
        if(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.8) cornerSquare();
    }

    private final Pose blueGoalPose = new Pose(15.5, 131.89); // 141x141 x:1.5-142.5 y:0-141 16.534

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

    String prevZone = null;

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

            if(hasSubsystem(SubsystemConfig.LL)){
                double now = runtime.getElapsedTimeSeconds();

                Pose odo = f.getPose();
                double speed = Double.NaN;
                double omega = Double.NaN;

                ekf.predictFromAbsoluteOdo(now, odo, speed, omega);

                ll.update(now, odo.getHeading());

                VisionMeasurement vm = ll.getMeasurement();
                if (vm != null) {
                    ekf.updateVisionWithLatency(
                            now,
                            vm.pose,
                            vm.timestampSec,
                            speed, omega,
                            vm.quality
                    );
                }

                f.setPose(ekf.getPose());
                f.update();
            }

            if(opModeType == OpModeType.TELEOP && hasSubsystems(List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER))){
                autoFireButton.input(g1.getButton(GamepadKeys.Button.CROSS));
                String zone = launchZones.firstHitName(chassisBox());
                if(autoFireButton.getVal()) {
                    if (zone != null && !zone.equals(prevZone) && !outtakeCommand.isScheduled() && storage.getSize() > 0) {
                        outtakeCommand.schedule();
                    }
                    if (zone == null && outtakeCommand.isScheduled())
                        CommandScheduler.getInstance().cancel(outtakeCommand);
                }
                t.addData("Current Zone", zone);
                prevZone = zone;
            }
            if(opModeType == OpModeType.TELEOP && hasSubsystem(SubsystemConfig.TURRET)){
                turretAlwaysReadyButton.input(g1.getButton(GamepadKeys.Button.CIRCLE));
                turret.setAlwaysAtTarget(turretAlwaysReadyButton.getVal());
            }
        }
        if(hasSubsystem(SubsystemConfig.INTAKE)){
            storage.inputAmps(intake.getCurrent());
            intake.inputStorageSize(storage.getSize());
        }
    }
    public void runIntakeTeleop(){
        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(new InstantCommand(() -> intake.setMode(Intake.Mode.REVERSE), intake));
    }

    public void runTransferTeleop(){
        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && storage.getSize() != 0)
            schedule(new TransferCommand(intake, storage));
    }

    public void runShootTeleop(){
        double lt = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rt = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if(lt > 0.1 || rt > 0.1) Turret.MANUAL_OFFSET += (rt - lt)/2;

        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && storage.getSize() != 0)
            schedule(outtakeCommand);
    }

    public void liftTeleop(){
        if(g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) schedule(new LiftCommand(intake, turret, shooter, lift));
    }

    public void endPeriodic() {
        this.run();
        if (hasSubsystem(SubsystemConfig.FOLLOWER)) {
            f.update();
            t.addData("Current Pose", f.getPose());
            t.addData("Current Velocity", f.getVelocity());
            t.addData("Follower Busy?" , f.isBusy());
        }
        if(hasSubsystems(List.of(SubsystemConfig.SHOOTER, SubsystemConfig.TURRET, SubsystemConfig.INTAKE, SubsystemConfig.DOOR))){
            t.addLine();
            t.addLine("SHOOTER DATA");
            if(!shooter.readyToShoot()) t.addLine("Shooter Not Ready");
            if(!turret.reachedTarget()) t.addLine("Turret Not Ready");
            if(!door.isOpen()) t.addLine("Door Not Open");
            t.addData("Shoot Ready?", shooter.readyToShoot() && turret.reachedTarget() && door.isOpen());
            t.addLine();
        }
        if(outtakeCommand != null){
            t.addData("Outtake Command Scheduled", outtakeCommand.isScheduled());
            t.addData("Outtake Command Done", outtakeCommand.isFinished());
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
            if(autoEndPose == null) autoEndPose = new Pose(chassisLeftOut + 1.25, chassisBack, Math.toRadians(90));
            this.f.setStartingPose(autoEndPose);
            this.f.update();
            this.ekf.resetPose(autoEndPose);
            if(opModeType == OpModeType.TELEOP) this.startDrive();
        }
    }

    public void setPose(Pose pose){
        f.setPose(pose);
        f.update();
        ekf.resetPose(pose);
    }

    public void cornerSquare(){
        if(isRed) setPose(new Pose(chassisLeftOut + 1.25, chassisBack, Math.toRadians(90)));
        else setPose(new Pose(144 - chassisRightOut - 1.25, chassisBack, Math.toRadians(90)));
    }

    private SAT2D.ConvexPolygon chassisBox(){
        final double x = f.getPose().getX();
        final double y = f.getPose().getY();
        final double h = f.getPose().getHeading();

        final double c = Math.cos(h);
        final double s = Math.sin(h);

        // Local corners (robot frame)
        final double[] lx = { +chassisFront, +chassisFront, -chassisBack,  -chassisBack  };
        final double[] ly = { +chassisLeft,  -chassisRight, -chassisRight, +chassisLeft };

        SAT2D.Point2 p0 = rotTrans(lx[0], ly[0], x, y, c, s);
        SAT2D.Point2 p1 = rotTrans(lx[1], ly[1], x, y, c, s);
        SAT2D.Point2 p2 = rotTrans(lx[2], ly[2], x, y, c, s);
        SAT2D.Point2 p3 = rotTrans(lx[3], ly[3], x, y, c, s);

        return new SAT2D.ConvexPolygon(p0, p1, p2, p3);
    }

    private static SAT2D.Point2 rotTrans(double lx, double ly, double x, double y, double c, double s) {
        double X = x + c * lx - s * ly;
        double Y = y + s * lx + c * ly;
        return new SAT2D.Point2(X, Y);
    }


    /// Commands ///

    public Command shootAll(){
        return new OuttakeCommand(intake, turret, shooter, door, storage);
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
}
