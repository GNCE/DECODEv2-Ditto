package org.firstinspires.ftc.teamcode.config.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Motif;
import org.firstinspires.ftc.teamcode.config.core.util.VisionMeasurement;

import java.util.List;

public class Limelight extends SubsysCore {
    Limelight3A ll;
    LLResult llResult;

    double tx, ty, ta;
    // Offset front: 198.49661mm
    // Offset Up: 217.62101mm
    // Offset Angle Pitch: 10.806923 deg up from vertical
    double nowSec;
    double robotHeading;
    VisionMeasurement latestMeasurement = null;

    public Limelight(){
        ll = h.get(Limelight3A.class, "Limelight");
        ll.setPollRateHz(50);
        ll.start();
        setMode(Mode.LOCALIZATION);
    }

    public static double LIMELIGHT_HEIGHT = 16.5; // Inches
    public static double APRILTAG_HEIGHT = 0;
    public static double CAMERA_PITCH = 15; // degrees from vertical

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

    public void update(double nowSec, double robotHeading){
        this.nowSec = nowSec;
        this.robotHeading = robotHeading;
    }

    public VisionMeasurement getMeasurement(){
        return latestMeasurement;
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
                ll.updateRobotOrientation(robotHeading);
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

            latestMeasurement = null;

            switch (mode){
                case LOCALIZATION:
                    if (llResult.isValid()) {
                        Pose3D bot = llResult.getBotpose_MT2();
                        if (bot != null) {
                            // Assumption: botpose units match your Pinpoint/Pedro units (inches).
                            // If you later find botpose is meters, convert here.

                            double x = bot.getPosition().x;
                            double y = bot.getPosition().y;
                            double headingRad = bot.getOrientation().getYaw(AngleUnit.RADIANS);

                            // Backdate measurement time by capture+targeting (ms -> s)
                            double tVision = nowSec - (captureLatency + targetingLatency) / 1000.0;

                            // Quality inflation (>=1): staleness, parse latency, etc.
                            double quality = 1.0;
                            long stale = llResult.getStaleness();
                            if (stale > 80) quality *= 2.0;
                            if (stale > 150) quality *= 5.0;
                            if (parseLatency > 15) quality *= 1.5;

                            latestMeasurement = new VisionMeasurement(
                                    new Pose(x, y, headingRad),
                                    tVision,
                                    quality
                            );

                            t.addData("MT2 Pose", "x=%.2f y=%.2f h=%.1fdeg", x, y, Math.toDegrees(headingRad));
                            t.addData("MT2 tVision", "%.3f", tVision);
                            t.addData("MT2 quality", "%.2f", quality);
                        }
                    }
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
