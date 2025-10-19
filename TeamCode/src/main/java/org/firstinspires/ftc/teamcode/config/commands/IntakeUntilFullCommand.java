package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.core.util.Artifact;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Spindex;

import java.util.Collections;

public class IntakeUntilFullCommand extends RepeatCommand {

    public IntakeUntilFullCommand(Intake intake, Door door, Spindex spindex) {
        super(new IntakeCommand(intake, door, spindex), spindex::isFull);
        addRequirements(intake, door, spindex);
    }
}