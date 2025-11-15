package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.core.util.Artifact;
import org.firstinspires.ftc.teamcode.config.core.util.ArtifactMatch;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Spindex;

public class RejectCommand extends SequentialCommandGroup {
    public RejectCommand(ArtifactMatch artifact, Intake intake, Spindex spindex, Door door) {
        addCommands(
                spindex.goToSlot(artifact),
                new WaitUntilCommand(spindex::reachedTarget),
                intake.setPowerInstant(Intake.IntakeMotorPowerConfig.REJECT),
                new WaitCommand(1000),
                new InstantCommand(spindex::removeItem)
        );
        addRequirements(intake, spindex, door);
    }
}
