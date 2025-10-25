package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.List;

@TeleOp(name="Brushland Analog Test", group="Brushland")
public class BrushlandAnalogTest extends OpMode {
    List<LynxModule> hubs;
    AnalogInput pin0;
    DigitalChannel pin1;
    @Override
    public void init() {
        pin0 = hardwareMap.analogInput.get("analog0");
        pin1 = hardwareMap.digitalChannel.get("digital1");
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub: hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        for(LynxModule hub: hubs){
            hub.clearBulkCache();
        }
        telemetry.addData("analog 0", pin0.getVoltage()/3.3*360);
        telemetry.addData("digital 1", pin1.getState());
        telemetry.update();
    }
}
