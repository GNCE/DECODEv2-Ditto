package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.core.util.ArtifactMatch;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

public class OuttakeAllCommand extends RepeatCommand {
    public OuttakeAllCommand(Intake intake, Spindex spindex, Turret turret, Shooter shooter, Door door) {
        super(new SequentialCommandGroup(new OuttakeCommand(ArtifactMatch.ANY, intake, spindex, turret, shooter, door), new WaitCommand(5)), spindex::isEmpty);
        addRequirements(intake, spindex, turret, shooter, door);
    }
}