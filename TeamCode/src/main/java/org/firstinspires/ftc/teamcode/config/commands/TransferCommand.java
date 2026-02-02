package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.config.core.util.ArtifactMatch;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(ArtifactMatch artifact, Spindex spindex, Door door, Intake intake){
        addCommands(
                intake.setPowerInstant(Intake.IntakeMotorPowerConfig.TRANSFER),
                new ParallelCommandGroup(
                        door.setOpenCommand(true),
                        spindex.goToSlot(artifact)
                ),
                intake.setUpCommand(true),
                intake.setUpCommand(false),
                new InstantCommand(spindex::removeItem)
        );
        addRequirements(spindex, door, intake);
    }
}
