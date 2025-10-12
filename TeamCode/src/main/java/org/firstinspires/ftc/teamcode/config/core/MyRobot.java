package org.firstinspires.ftc.teamcode.config.core;

import com.seattlesolvers.solverslib.command.Robot;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class MyRobot extends Robot {
    HardwareMap h;
    JoinedTelemetry t;
    Follower f;

    List<LynxModule> hubs;
    LoopTimer lt;

    public MyRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2){
        this.h = h;
        this.t = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), t);
        hubs = this.h.getAll(LynxModule.class);
        for(LynxModule hub: hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        this.lt = new LoopTimer();
    }

    public void resetCache(){
        for(LynxModule hub: hubs){
            hub.clearBulkCache();
        }
    }

    public void startPeriodic(){
        lt.start();
        resetCache();
    }
    public void endPeriodic(){
        this.run();
        lt.end();
        t.addData("Loop Time (ms)", lt.getMs());
        t.addData("Loop Frequency (Hz)", lt.getHz());
        t.update();
    }
}
