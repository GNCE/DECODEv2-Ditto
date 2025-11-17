package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Spindex;

public class SafeReverseCommand extends SequentialCommandGroup {
    public SafeReverseCommand(Intake intake, Spindex spindex) {
        super(
                new InstantCommand(() -> spindex.setSafetyMode(true)),
                new WaitUntilCommand(spindex::reachedTarget),
                intake.setPowerInstant(Intake.IntakeMotorPowerConfig.REJECT),
                new WaitCommand(1500),
                new InstantCommand(() -> spindex.setSafetyMode(false)),
                new WaitUntilCommand(spindex::reachedTarget)
        );
        addRequirements(intake, spindex);
    }
}
