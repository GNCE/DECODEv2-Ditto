package org.firstinspires.ftc.teamcode.config.core;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;

import java.util.List;

/**
 * Identical to {@link MyRobot} in every way EXCEPT it binds gamepad-1 DPAD_RIGHT to an auto-park:
 * the robot drives from wherever it is to a fixed park pose. The park assumes the robot is
 * stationary when triggered (a simple straight line from the current pose, no velocity lead).
 *
 * <p>Park target (y and heading identical for both alliances; only x flips):
 * <ul>
 *   <li>RED  -> ({@link #PARK_X_RED}, {@link #PARK_Y}, {@link #PARK_HEADING_DEG})</li>
 *   <li>BLUE -> (fieldSize - {@link #PARK_X_RED}, {@link #PARK_Y}, {@link #PARK_HEADING_DEG})</li>
 * </ul>
 *
 * <p>Because it extends MyRobot, it drops straight into any field typed {@code MyRobot r;}:
 * <pre>r = new MyRobot_AutoPark(hardwareMap, telemetry, gamepad1, gamepad2, ...);</pre>
 */
@Configurable
public class MyRobot_AutoPark extends MyRobot {

    // Park pose. RED uses PARK_X_RED; BLUE uses (fieldSize - PARK_X_RED). y/heading are shared.
    public static double PARK_X_RED = 40;
    public static double PARK_Y = 34;
    public static double PARK_HEADING_DEG = 270;

    // If any drive stick exceeds this while parking, the driver is taking over: cancel the park.
    public static double STICK_CANCEL_DEADBAND = 0.1;

    private boolean autoParking = false;

    // --- Constructors just forward to MyRobot; all real init happens in the superclass. ---
    public MyRobot_AutoPark(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, List<SubsystemConfig> subsysList, OpModeType opModeType){
        super(h, t, g1, g2, subsysList, opModeType);
    }

    public MyRobot_AutoPark(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, List<SubsystemConfig> subsysList){
        super(h, t, g1, g2, subsysList);
    }

    public MyRobot_AutoPark(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, OpModeType opModeType){
        super(h, t, g1, g2, opModeType);
    }

    /** Alliance-correct park pose. Only x flips between alliances; y and heading are shared. */
    public Pose autoParkPose(){
        double x = (isRed != null && isRed) ? PARK_X_RED : (fieldSize - PARK_X_RED);
        return new Pose(x, PARK_Y, Math.toRadians(PARK_HEADING_DEG));
    }

    /** Build a straight line from the current pose to the park pose and start following it. */
    private void startAutoPark(){
        if(!hasSubsystem(SubsystemConfig.FOLLOWER)) return;
        Pose target = autoParkPose();
        f.followPath(
                f.pathBuilder()
                        .addPath(new BezierLine(f.getPose(), target))
                        .setLinearHeadingInterpolation(f.getHeading(), target.getHeading())
                        .build(),
                true); // holdEnd: keep holding the park pose once arrived
        autoParking = true;
    }

    /**
     * Same as {@link MyRobot#driveControls()} plus a DPAD_RIGHT auto-park. While parking, the
     * teleop drive call is suppressed so it doesn't fight the path; the park ends when the path
     * finishes or the driver moves a stick (manual override). The other g1 bindings are skipped
     * while parking so a stray press can't interrupt it.
     */
    @Override
    public void driveControls(){
        boolean justStarted = false;

        if(hasSubsystem(SubsystemConfig.FOLLOWER)){
            // Start auto-park on DPAD_RIGHT.
            if(g1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                startAutoPark();
                justStarted = true;
            }

            // While parking (and not on the very loop we started), end it if the driver grabs the
            // sticks or the path completes, then hand control back to teleop drive.
            if(autoParking && !justStarted){
                boolean driverInput =
                        Math.abs(g1.getLeftY())  > STICK_CANCEL_DEADBAND ||
                                Math.abs(g1.getLeftX())  > STICK_CANCEL_DEADBAND ||
                                Math.abs(g1.getRightX()) > STICK_CANCEL_DEADBAND;
                if(driverInput || !f.isBusy()){
                    autoParking = false;
                    f.startTeleopDrive(); // resume normal driver control
                }
            }

            // Only drive from the sticks when NOT parking, otherwise we'd override the path.
            if(!autoParking){
                f.setTeleOpDrive(g1.getLeftY(), -g1.getLeftX(), -g1.getRightX(), true);
            }
        }

        // Existing MyRobot g1 bindings, unchanged. Skipped while parking.
        if(!autoParking){
            if(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.8) cornerSquare();
            if(g1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) alignPreset();
            if(g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) farWallSquare();
            if(g1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) allianceWallSquare();
        }

        t.addData("Auto Parking", autoParking);
    }
}