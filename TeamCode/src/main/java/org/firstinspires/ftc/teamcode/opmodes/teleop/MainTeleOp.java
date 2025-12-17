package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.SubsystemConfig;

import java.util.List;

@Configurable
@TeleOp(group="TeleOp", name="Main TeleOp")
public class MainTeleOp extends MyCommandOpMode {

    public static double targetVel = 0;
    public static double hoodAngle = 55;
    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.LIFT, SubsystemConfig.INTAKE, SubsystemConfig.SPINDEX, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER));
    }

    @Override
    public void initialize_loop() {
        r.preloadSelection();
    }

    @Override
    public void run() {
        r.driveControls();
        r.runIntakeTeleop();
        r.runShootTeleop();
        r.liftTeleop();
        r.shooter.setVelocity(targetVel);
        r.shooter.setHoodAngle(hoodAngle);
    }
}
