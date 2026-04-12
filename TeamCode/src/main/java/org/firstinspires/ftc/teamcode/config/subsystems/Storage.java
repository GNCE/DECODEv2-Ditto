package org.firstinspires.ftc.teamcode.config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.core.util.hardware.ModeSmoother;

@Configurable
public class Storage extends SubsysCore {
    private final DigitalChannel s1, s2, s3;
    private final DigitalChannel p1, p2, p3;

    private final ModeSmoother<Integer> beam1Smoother;
    private final ModeSmoother<Integer> beam2Smoother;
    private final ModeSmoother<Integer> beam3Smoother;

    private boolean st1, st2, st3;
    private int storedCount = 0;

    private int assumeS3FaultLoops = 0;
    private int assumeS2FaultLoops = 0;

    public static int beamSmoothWindow = 5;
    public static int faultyBeamHoldLoops = 50;

    public static double currentLimit = 4;
    public static double veloLimit = 40;

    private double curAmps = 0;
    private double curVelo = 0;

    Timer timer;

    public Storage() {
        s1 = h.get(DigitalChannel.class, "s1");
        s2 = h.get(DigitalChannel.class, "s2");
        s3 = h.get(DigitalChannel.class, "s3");

        p1 = h.get(DigitalChannel.class, "p1");
        p2 = h.get(DigitalChannel.class, "p2");
        p3 = h.get(DigitalChannel.class, "p3");

        s1.setMode(DigitalChannel.Mode.INPUT);
        s2.setMode(DigitalChannel.Mode.INPUT);
        s3.setMode(DigitalChannel.Mode.INPUT);

        p1.setMode(DigitalChannel.Mode.OUTPUT);
        p2.setMode(DigitalChannel.Mode.OUTPUT);
        p3.setMode(DigitalChannel.Mode.OUTPUT);

        p1.setState(true);
        p2.setState(true);
        p3.setState(true);

        beam1Smoother = new ModeSmoother<>(beamSmoothWindow, 0);
        beam2Smoother = new ModeSmoother<>(beamSmoothWindow, 0);
        beam3Smoother = new ModeSmoother<>(beamSmoothWindow, 0);

        storedCount = 0;
        storageState = StorageState.WAIT_FOR_ST3;

        timer = new Timer();
    }

    public void input(double curAmps, double curVelo) {
        this.curAmps = curAmps;
        this.curVelo = Math.abs(curVelo);
    }

    public void clear() {
        storedCount = 0;
        assumeS3FaultLoops = 0;
        assumeS2FaultLoops = 0;
    }

    public int getSize() {
        return storedCount;
    }

    public boolean isFull() {
        return storedCount >= 3;
    }

    @Override
    public void periodic() {
        readSmoothedBeams();
        updateStoredCount();

        t.addData("Stored Count", storedCount);
        t.addData("Break Beam 1", st1);
        t.addData("Break Beam 2", st2);
        t.addData("Break Beam 3", st3);
        t.addData("Assume S3 Fault Loops", assumeS3FaultLoops);
        t.addData("Assume S2 Fault Loops", assumeS2FaultLoops);
    }

    private void readSmoothedBeams() {
        beam1Smoother.add(!s1.getState() ? 1 : 0);
        beam2Smoother.add(!s2.getState() ? 1 : 0);
        beam3Smoother.add(!s3.getState() ? 1 : 0);

        st1 = beam1Smoother.mode() == 1;
        st2 = beam2Smoother.mode() == 1;
        st3 = beam3Smoother.mode() == 1;
    }

    private void updateStoredCount() {
        updateNormalTransitions();
        updateFaultFallbacks();
    }

    enum StorageState {
        WAIT_FOR_ST3,
        DELAY_AFTER_ST3,
        WAIT_FOR_ST2,
        DELAY_AFTER_ST2,
        WAIT_FOR_ST1,
        DELAY_AFTER_ST1,
        FULL
    }

    StorageState storageState;
    public static double ST3_DELAY = 0.25, ST2_DELAY = 0.25, ST1_DELAY = 0.25;

    private void updateNormalTransitions() {
        switch(storageState) {
            case WAIT_FOR_ST3:
                if (st3) {
                    timer.resetTimer();
                    storageState = StorageState.DELAY_AFTER_ST3;
                }
                break;
            case DELAY_AFTER_ST3:
                if(timer.getElapsedTimeSeconds() > ST3_DELAY){
                    storedCount = 1;
                    storageState = StorageState.WAIT_FOR_ST2;
                }
                break;
            case WAIT_FOR_ST2:
                if (st2) {
                    timer.resetTimer();
                    storageState = StorageState.DELAY_AFTER_ST2;
                }
                break;
            case DELAY_AFTER_ST2:
                if(timer.getElapsedTimeSeconds() > ST2_DELAY){
                    storedCount = 2;
                    storageState = StorageState.WAIT_FOR_ST1;
                }
                break;
            case WAIT_FOR_ST1:
                if (st1) {
                    timer.resetTimer();
                    storageState = StorageState.DELAY_AFTER_ST1;
                }
                break;
            case DELAY_AFTER_ST1:
                if(timer.getElapsedTimeSeconds() > ST1_DELAY){
                    storedCount = 3;
                    storageState = StorageState.FULL;
                }
                break;
            case FULL:
                break;
        }
    }

    private void updateFaultFallbacks() {
        updateS3Fallback();
        updateS2Fallback();
    }

    private void updateS3Fallback() {
        if (storedCount == 0 && !st3 && (st2 || st1)) {
            assumeS3FaultLoops++;
            if (assumeS3FaultLoops >= faultyBeamHoldLoops) {
                storedCount = 2;
                storageState = StorageState.WAIT_FOR_ST1;
                assumeS3FaultLoops = 0;
            }
        } else {
            assumeS3FaultLoops = 0;
        }
    }

    private void updateS2Fallback() {
        if (st3 && !st2 && st1) {
            assumeS2FaultLoops++;
            if (assumeS2FaultLoops >= faultyBeamHoldLoops) {
                storedCount = 3;
                storageState = StorageState.FULL;
                assumeS2FaultLoops = 0;
            }
        } else {
            assumeS2FaultLoops = 0;
        }
    }
}