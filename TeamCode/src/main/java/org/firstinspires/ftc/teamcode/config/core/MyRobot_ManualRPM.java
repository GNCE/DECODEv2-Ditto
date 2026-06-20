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
 * two presets, toggled by gamepad-2 RIGHT STICK BUTTON (R3):
 * <ul>
 *   <li>CLOSE = {@link #CLOSE_PRESPIN_RPM} (the default at startup)</li>
 *   <li>FAR   = {@link #FAR_PRESPIN_RPM}</li>
 * </ul>
 * The selected RPM is applied via the shooter's spin-up hold, which holds it outright (so it
 * overrides the inherited live distance target) until you toggle to the other preset. R3 flips
 * between the two at any time during the match.
 *
 * <p>Because it extends MyRobot, it drops straight into any field typed {@code MyRobot r;}:
 * <pre>r = new MyRobot_ManualRPM(hardwareMap, telemetry, gamepad1, gamepad2, ...);</pre>
 * Everything else (drive, intake, auto-fire, lift, etc.) is inherited unchanged.
 */
@Configurable
public class MyRobot_ManualRPM extends MyRobot {

    // Two manual flywheel RPM presets. Toggled with g2 R3; defaults to CLOSE at startup.
    public static double CLOSE_PRESPIN_RPM = 1500;
    public static double FAR_PRESPIN_RPM = 2000;

    private boolean farMode = false; // false = CLOSE (default at init), true = FAR
    private boolean seeded  = false; // apply the default CLOSE baseline on the first teleop loop

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

    /**
     * Same as {@link MyRobot#runShootTeleop()} plus the CLOSE/FAR manual RPM toggle on g2 R3.
     * Starts in CLOSE; each R3 press flips to the other preset and (re)applies that RPM as the
     * shooter's spin-up hold. setSpinUpRpm is a latching hold, so one call per change is enough.
     * Firing is still gated on shooter.readyToShoot() via OuttakeCommandTele.
     */
    @Override
    public void runShootTeleop(){
        double lt = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rt = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if(lt > 0.1 || rt > 0.1) Turret.MANUAL_OFFSET += (rt - lt)/2;

        if(hasSubsystem(SubsystemConfig.SHOOTER)){
            boolean toggled = g2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON);
            if(toggled) farMode = !farMode;

            // Seed CLOSE on the first loop, then re-apply only when the mode actually flips.
            if(toggled || !seeded){
                shooter.setSpinUpRpm(farMode ? FAR_PRESPIN_RPM : CLOSE_PRESPIN_RPM);
                seeded = true;
            }
        }

        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) //  && storage.getSize() != 0
            schedule(OuttakeCommandTele);

        t.addData("Manual RPM Mode", farMode ? ("FAR (" + FAR_PRESPIN_RPM + ")") : ("CLOSE (" + CLOSE_PRESPIN_RPM + ")"));
    }
}