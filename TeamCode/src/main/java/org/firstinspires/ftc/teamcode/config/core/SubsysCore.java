package org.firstinspires.ftc.teamcode.config.core;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubsysCore extends SubsystemBase {
    public static HardwareMap h;
    public static JoinedTelemetry t;

    public static void setGlobalParameters(HardwareMap newH, JoinedTelemetry newT) {
        h = newH;
        t = newT;
    }
}
