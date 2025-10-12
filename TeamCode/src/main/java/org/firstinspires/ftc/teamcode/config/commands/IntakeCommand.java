package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.config.core.util.Artifact;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Spindex;

import java.util.Arrays;
import java.util.Collections;

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
                        new DeferredCommand(() -> door.setOpenCommand(false), Collections.singletonList(door)),
                        new DeferredCommand(() -> spindex.goToSlot(Artifact.NONE), Collections.singletonList(spindex))
                ),
                new InstantCommand(() -> intake.setPower(Intake.IntakeMotorPowerConfig.INTAKE)),
                new WaitUntilCommand(() -> intake.getCurrentArtifact() != Artifact.NONE),
                new InstantCommand(() -> spindex.insertItem(intake.getCurrentArtifact()))
        );
        addRequirements(intake, door, spindex);
    }

    @Override
    public void initialize() {
        super.initialize();
        if(!spindex.contains(Artifact.NONE)) cancel();
    }
}
