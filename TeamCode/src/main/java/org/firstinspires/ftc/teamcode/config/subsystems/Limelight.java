package org.firstinspires.ftc.teamcode.config.subsystems;

import android.widget.Switch;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.core.util.Motif;

import java.util.List;

public class Limelight extends SubsysCore {
    Limelight3A ll;
    LLResult llResult;

    double tx, ty, ta;

    public Limelight(){
        ll = h.get(Limelight3A.class, "Limelight");
        ll.setPollRateHz(100);
        ll.start();
        setMode(Mode.LOCALIZATION);
    }

    public static double LIMELIGHT_HEIGHT = 16.5; // Inches
    public static double APRILTAG_HEIGHT = 0;
    public static double CAMERA_PITCH = 15; // degrees from vertical

    public double getDistanceInches(){
        return (APRILTAG_HEIGHT-LIMELIGHT_HEIGHT)/Math.tan(Math.toRadians(CAMERA_PITCH + ty));
    }

    public boolean isDataValid(){
        return llResult != null && llResult.isValid();
    }

    public boolean isAprilTagDetected(){ return llResult !=null && llResult.isValid(); }

    public enum Mode {
        LOCALIZATION,
        MOTIF_DETECTION
    }

    Mode mode;

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    @Override
    public void periodic() {
        LLStatus status = ll.getStatus();
        t.addData("LL Name", "%s", status.getName());
        t.addData("LL State", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(),(int)status.getFps());
        t.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());
        t.addData("LL Mode", mode.name());

        switch (mode){
            case LOCALIZATION:
                ll.pipelineSwitch(1);
                break;
            case MOTIF_DETECTION:
                ll.pipelineSwitch(0);
                break;
        }

        llResult = ll.getLatestResult();

        if (llResult != null) {
            double captureLatency = llResult.getCaptureLatency();
            double targetingLatency = llResult.getTargetingLatency();
            double parseLatency = llResult.getParseLatency();
            t.addData("LL Latency", captureLatency + targetingLatency);
            t.addData("LL Parse Latency", parseLatency);

            tx = llResult.getTx();
            ty = llResult.getTy();
            ta = llResult.getTa();

            t.addData("Target X", tx);
            t.addData("Target Y", ty);
            t.addData("Target Area", ta);

            switch (mode){
                case LOCALIZATION:
                    break;
                case MOTIF_DETECTION:
                    if(MyRobot.currentMotif == null){
                        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
                        if(fiducials.size() == 1){
                            int id = fiducials.get(0).getFiducialId();
                            MyRobot.currentMotif = Motif.getMotif(id);
                        }
                    }
                    break;
            }
        } else {
            t.addLine("AprilTag Not Detected");
        }

        t.addData("Limelight Connected?", ll.isConnected());
        t.addData("Limelight Running?", ll.isRunning());
    }
}
