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
 * Identical to {@link MyRobot} in every way EXCEPT it adds manual flywheel pre-spin on the
 * gamepad-2 bumpers: LB pre-spins to {@link #CLOSE_PRESPIN_RPM}, RB to {@link #FAR_PRESPIN_RPM}.
 * Because it extends MyRobot, it drops straight into any field typed {@code MyRobot r;}:
 * <pre>r = new MyRobot_ManualRPM(hardwareMap, telemetry, gamepad1, gamepad2, ...);</pre>
 * Everything else (drive, intake, auto-fire, lift, the per-loop distance RPM/hood solve, etc.)
 * is inherited unchanged.
 */
@Configurable
public class MyRobot_ManualRPM extends MyRobot {

    // Manual flywheel pre-spin baselines (g2 bumpers). LB = close, RB = far. Releasing the bumper
    // hands the flywheel back to the live distance target, which then fine-tunes the final RPM.
    public static double CLOSE_PRESPIN_RPM = 1500;
    public static double FAR_PRESPIN_RPM = 2000;

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
     * Same as {@link MyRobot#runShootTeleop()} plus manual RPM pre-spin on the g2 bumpers.
     * Hold g2 LEFT BUMPER to spin the flywheel up to the CLOSE baseline, g2 RIGHT BUMPER for the
     * FAR baseline. From there the inherited per-loop distance logic still runs, so the wheel only
     * fine-tunes around the baseline instead of ramping from idle. Releasing either bumper (when
     * neither is held) clears the hold and hands the wheel back to the live distance target.
     * Firing is still gated on shooter.readyToShoot() via OuttakeCommandTele.
     */
    @Override
    public void runShootTeleop(){
        double lt = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rt = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if(lt > 0.1 || rt > 0.1) Turret.MANUAL_OFFSET += (rt - lt)/2;

        if(hasSubsystem(SubsystemConfig.SHOOTER)){
            if(g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) shooter.setSpinUpRpm(CLOSE_PRESPIN_RPM);
            else if(g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) shooter.setSpinUpRpm(FAR_PRESPIN_RPM);
            else if((g2.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER) || g2.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER))
                    && !g2.getButton(GamepadKeys.Button.LEFT_BUMPER) && !g2.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                shooter.clearSpinUp();
        }

        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) //  && storage.getSize() != 0
            schedule(OuttakeCommandTele);
    }
}