package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;

public class IntakeUntilFullCommand extends RepeatCommand {

    public IntakeUntilFullCommand(Intake intake, Door door, Spindex spindex) {
        super(new SequentialCommandGroup(new IntakeCommand(intake, door, spindex), new WaitCommand(5)), spindex::isFull);
        addRequirements(intake, door, spindex);
    }
}