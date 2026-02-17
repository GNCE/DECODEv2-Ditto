package org.firstinspires.ftc.teamcode.config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.core.util.hardware.ModeSmoother;

@Configurable
public class Storage extends SubsysCore{
    DigitalChannel s1, s2, s3, p1, p2, p3; // Numbered from Top(Turret) to Bottom(Intake)
    boolean st1, st2, st3; // States
    ModeSmoother<Integer> sizeContainer;
    int cnt = 0;
    double curAmps = 0;
    public static double currentLimit = 4;

    public Storage(){
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

        sizeContainer = new ModeSmoother<>(10, 0);
    }

    public void inputAmps(double cur){
        curAmps = cur;
    }

    @Override
    public void periodic() {
        st1 = !s1.getState();
        st2 = !s2.getState();
        st3 = !s3.getState();

        cnt = 0;
        if(st3){
            cnt++;
            if(st2) {
                cnt++;
                if(st1) cnt++;
            }
        }

        if(curAmps > 4) cnt = 3;

        sizeContainer.add(cnt);

        t.addData("Ball Count", cnt);
        t.addData("Break Beam 1", st1);
        t.addData("Break Beam 2", st2);
        t.addData("Break Beam 3", st3);
    }

    public int getSize(){
        return sizeContainer.mode();
    }
}