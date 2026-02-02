package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.core.util.ArtifactMatch;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;

public class TransferAllCommand extends RepeatCommand {
    public TransferAllCommand(Intake intake, Spindex spindex,  Door door) {
        super(new SequentialCommandGroup(new TransferCommand(ArtifactMatch.ANY, spindex, door, intake), new WaitCommand(5)), spindex::isEmpty);
        addRequirements(intake, spindex, door);
    }
}