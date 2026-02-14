package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.core.util.ArtifactMatch;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Lift;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Storage;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

public class LiftCommand extends SequentialCommandGroup {
    public LiftCommand(Intake intake, Turret turret, Shooter shooter, Lift lift){
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setMode(Intake.Mode.DISABLE)),
                        // new InstantCommand(() -> turret.) TODO: Disable
                        new InstantCommand(() -> shooter.setActive(false))
                ),
                lift.EngageClutchCommand()
        );
        addRequirements(intake, turret, shooter, lift);
    }
}
