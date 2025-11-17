package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.OpModeType;

@TeleOp(group="TeleOp", name="Main TeleOp")
public class MainTeleOp extends CommandOpMode {
    MyRobot r;

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, OpModeType.TELEOP);
    }

    @Override
    public void initialize_loop() {
        r.startInitLoop();
        r.endInitLoop();
    }

    @Override
    public void run() {
        r.startPeriodic();
        r.runIntakeTeleop();
        r.endPeriodic();
    }

    @Override
    public void end() {
        r.stop();
    }
}
