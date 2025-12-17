package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.subsystems.Lift;

import java.util.List;

@TeleOp(name="Lift Manual", group="Utils")
public class LiftManualControl extends MyCommandOpMode {
    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.LIFT));
        r.lift.setMode(Lift.Mode.MANUAL);
    }

    @Override
    public void run() {
        r.lift.setManualPower(r.g1.getLeftY());
        r.liftTeleop();
    }
}
