package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.core.util.Artifact;
import org.firstinspires.ftc.teamcode.config.core.util.ArtifactMatch;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(ArtifactMatch artifact, Spindex spindex, Door door, Intake intake){
        addCommands(
                intake.setPowerInstant(Intake.IntakeMotorPowerConfig.TRANSFER),
                new ParallelCommandGroup(
                        door.setOpenCommand(false),
                        spindex.goToSlot(artifact)
                ),
                intake.setUpCommand(true),
                intake.setUpCommand(false),
                new InstantCommand(spindex::removeItem)
        );
        addRequirements(spindex, door, intake);
    }
}
