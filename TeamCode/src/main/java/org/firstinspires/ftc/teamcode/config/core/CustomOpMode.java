package org.firstinspires.ftc.teamcode.config.core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class CustomOpMode extends OpMode {
    MyRobot r;

    public void init(){
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, 0);
    }

    @Override
    public void init_loop() {
        r.allianceSelection();
    }

    @Override
    public void loop() {
        r.startPeriodic();
        repeat();
        r.endPeriodic();
    }
    abstract void repeat();
}
