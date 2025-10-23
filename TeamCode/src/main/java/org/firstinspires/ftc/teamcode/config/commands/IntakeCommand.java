package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.config.core.util.Artifact;
import org.firstinspires.ftc.teamcode.config.core.util.ArtifactMatch;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Spindex;

public class IntakeCommand extends SequentialCommandGroup {
    Intake intake;
    Door door;
    Spindex spindex;

    public IntakeCommand(Intake intake, Door door, Spindex spindex) {
        this.intake = intake;
        this.door = door;
        this.spindex = spindex;
        addCommands(
                new ParallelCommandGroup(
                        door.setOpenCommand(false),
                        spindex.goToSlot(ArtifactMatch.NONE),
                        intake.setPowerInstant(Intake.IntakeMotorPowerConfig.STOP)
                ),
                intake.resetSmootherCommand(),
                intake.runUntilArtifactSensed(),
                intake.setPowerInstant(Intake.IntakeMotorPowerConfig.STOP),
                new InstantCommand(() -> spindex.insertItem(intake.getCurrentArtifact())),
                intake.resetSmootherCommand()
        );
        addRequirements(intake, door, spindex);
    }

    @Override
    public void initialize() {
        super.initialize();
        if(spindex.isFull()) cancel();
    }
}
