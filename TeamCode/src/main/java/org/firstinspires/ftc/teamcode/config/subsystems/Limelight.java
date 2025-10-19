package org.firstinspires.ftc.teamcode.config.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.core.SubsysCore;

public class Limelight extends SubsysCore {
    Limelight3A ll;
    Follower f;
    LLResult result;

    public Limelight(Follower ff, boolean isRed){
        this.f = ff;
        ll = h.get(Limelight3A.class, "Limelight");
        ll.setPollRateHz(50);
        ll.pipelineSwitch(isRed ? 1 : 2);
    }

    public boolean isAprilTagDetected(){ return result!=null && result.isValid(); }

    @Override
    public void periodic() {
        LLStatus status = ll.getStatus();
        t.addData("LL Name", "%s", status.getName());
        t.addData("LL State", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(),(int)status.getFps());
        t.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());

        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            // Access general information
            Pose3D botpose = result.getBotpose_MT2();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            t.addData("LL Latency", captureLatency + targetingLatency);
            t.addData("LL Parse Latency", parseLatency);

            t.addData("Target X", result.getTx());
            t.addData("Target Area", result.getTa());
            t.addData("Botpose", botpose.toString());
        } else {
            t.addLine("AprilTag Not Detected");
        }

        t.addData("Limelight Connected?", ll.isConnected());
        t.addData("Limelight Running?", ll.isRunning());
    }
}
