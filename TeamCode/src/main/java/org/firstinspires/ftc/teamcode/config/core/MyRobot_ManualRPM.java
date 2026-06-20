package org.firstinspires.ftc.teamcode.config.core;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

import java.util.List;

/**
 * Identical to {@link MyRobot} in every way EXCEPT it adds a manual flywheel pre-spin mode with
 * two presets, toggled by gamepad-1 DPAD_RIGHT:
 * <ul>
 *   <li>CLOSE = {@link #CLOSE_PRESPIN_RPM} (the default at startup)</li>
 *   <li>FAR   = {@link #FAR_PRESPIN_RPM}</li>
 * </ul>
 * The same button works during INIT (to pre-select before the match) and during the match. The
 * selected preset is shown on telemetry in both phases. The chosen RPM is applied via the
 * shooter's spin-up hold (which holds it outright, overriding the inherited live distance target)
 * on the first teleop loop, then re-applied whenever you toggle.
 *
 * <p>NOTE: the init-time toggle + telemetry live in {@link #startInitLoop()}, so the OpMode's
 * init loop must call {@code startInitLoop()} / {@code endInitLoop()} (endInitLoop flushes
 * telemetry). No flywheel motion happens during init -- init only selects the mode.
 *
 * <p>Because it extends MyRobot, it drops straight into any field typed {@code MyRobot r;}:
 * <pre>r = new MyRobot_ManualRPM(hardwareMap, telemetry, gamepad1, gamepad2, ...);</pre>
 */
@Configurable
public class MyRobot_ManualRPM extends MyRobot {

    // Two manual flywheel RPM presets. Toggled with g1 DPAD_RIGHT; defaults to CLOSE at startup.
    public static double CLOSE_MIN_RPM = 1400;
    public static double CLOSE_MAX_RPM = 1900;

    public static double FAR_MIN_RPM = 2000;
    public static double FAR_MAX_RPM = 2200;

    private boolean farMode = false; // false = CLOSE (default at init), true = FAR
    private boolean seeded  = false; // apply the selected baseline on the first teleop loop

    // --- Constructors just forward to MyRobot; all real init happens in the superclass. ---
    public MyRobot_ManualRPM(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, List<SubsystemConfig> subsysList, OpModeType opModeType){
        super(h, t, g1, g2, subsysList, opModeType);
    }

    public MyRobot_ManualRPM(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, List<SubsystemConfig> subsysList){
        super(h, t, g1, g2, subsysList);
    }

    public MyRobot_ManualRPM(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, OpModeType opModeType){
        super(h, t, g1, g2, opModeType);
    }

    private String modeLabel(){
        return farMode
                ? ("FAR [" + FAR_MIN_RPM + "-" + FAR_MAX_RPM + "]")
                : ("CLOSE [" + CLOSE_MIN_RPM + "-" + CLOSE_MAX_RPM + "]");
    }

    /**
     * Pre-match: let the driver flip CLOSE/FAR with g1 DPAD_RIGHT (the same button used in-match)
     * and show the current selection on telemetry. Does NOT spin the flywheel -- it only selects
     * the mode that the first teleop loop will apply. Relies on super.startInitLoop() having just
     * called g1.readButtons(), and on endInitLoop() to flush telemetry with t.update().
     */
    @Override
    public void startInitLoop(){
        super.startInitLoop();
        if(g1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) farMode = !farMode;
        t.addData("Manual RPM Mode (DPAD_RIGHT to toggle)", modeLabel());
    }

    /**
     * Same as {@link MyRobot#runShootTeleop()} plus the CLOSE/FAR manual RPM toggle on g1
     * DPAD_RIGHT. Carries over whatever was selected during init; each press flips the preset and
     * (re)applies that RPM as the shooter's spin-up hold. setSpinUpRpm is a latching hold, so one
     * call per change is enough. Firing is still gated on shooter.readyToShoot() via OuttakeCommandTele.
     */
    @Override
    public void runShootTeleop(){
        double lt = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rt = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if(lt > 0.1 || rt > 0.1) Turret.MANUAL_OFFSET += (rt - lt)/2;

        if(hasSubsystem(SubsystemConfig.SHOOTER)){
            boolean toggled = g1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT);
            if(toggled) farMode = !farMode;

            // Seed the selected preset on the first loop, then re-apply only when it flips.
            if (toggled || !seeded) {
                if (farMode) {
                    shooter.setRpmClamp(FAR_MIN_RPM, FAR_MAX_RPM);
                } else {
                    shooter.setRpmClamp(CLOSE_MIN_RPM, CLOSE_MAX_RPM);
                }
                seeded = true;
            }
        }

        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) //  && storage.getSize() != 0
            schedule(OuttakeCommandTele);

        t.addData("Manual RPM Mode", modeLabel());
    }
}