package org.firstinspires.ftc.teamcode.config.subsystems;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;

@Configurable
public class Door extends SubsysCore {
    Servo door;
    public static double OFFSET_POS = 0;
    public static double OPEN_POS = 0.77;
    public static double CLOSED_POS = 0.535;
    public static long ACTUATION_TIME_MS = 500;

    boolean open;

    public Door() {
        door = h.get(Servo.class, "door");
        open = false;
    }

    public boolean isOpen(){ return open; }

    public void setOpen(boolean state){ open = state; }


    public Command setOpenCommand(boolean state){
        return new ConditionalCommand(
                new WaitCommand(ACTUATION_TIME_MS),
                new InstantCommand(),
                () -> {
                    boolean changed = (open != state);
                    open = state;
                    return changed;
                }
        );
    }

    @Override
    public void periodic() {
        door.setPosition(OFFSET_POS + (open ? OPEN_POS : CLOSED_POS));
        t.addData("Door State", open ? "Open" : "Closed");
    }
}
