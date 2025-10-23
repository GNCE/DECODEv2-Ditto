package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.core.util.ArtifactMatch;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

import java.util.function.Predicate;

public class OuttakeCommand extends SequentialCommandGroup {
    public OuttakeCommand(ArtifactMatch artifactMatch, Intake intake, Spindex spindex,Turret turret, Shooter shooter, Door door){
        addCommands(
                new InstantCommand(shooter::turnOn),
                new ParallelCommandGroup(
                        door.setOpenCommand(false),
                        spindex.goToSlot(artifactMatch),
                        new WaitUntilCommand(turret::reachedTarget),
                        new WaitUntilCommand(shooter::readyToShoot)
                ),
                intake.setUpCommand(true),
                intake.setUpCommand(false),
                new InstantCommand(spindex::removeItem)
        );
        addRequirements(spindex, door, shooter, intake);
    }
}
