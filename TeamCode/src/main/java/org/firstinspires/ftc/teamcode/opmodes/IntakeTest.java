package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.IntakeRobot;

@TeleOp(name = "Intake Test", group = "System Test")
public class IntakeTest extends OpMode {
    IntakeRobot r;

    @Override
    public void init() {
        r = new IntakeRobot(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void init_loop() {
        r.startPeriodic();
        r.allianceSelection();
        r.preloadSelection();
        r.endPeriodic();
    }

    @Override
    public void loop() {
        r.startPeriodic();
        r.runIntakeTeleop();
        r.endPeriodic();
    }
}
