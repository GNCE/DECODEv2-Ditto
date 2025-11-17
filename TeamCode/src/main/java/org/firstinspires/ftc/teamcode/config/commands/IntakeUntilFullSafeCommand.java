package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Spindex;

public class IntakeUntilFullSafeCommand extends SequentialCommandGroup {
    public IntakeUntilFullSafeCommand(Intake intake, Door door, Spindex spindex) {
        super(new IntakeUntilFullCommand(intake, door, spindex), new SafeReverseCommand(intake, spindex));
        addRequirements(intake, door, spindex);
    }
}
