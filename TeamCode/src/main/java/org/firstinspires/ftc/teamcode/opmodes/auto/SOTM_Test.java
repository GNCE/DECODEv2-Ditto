package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Alliance;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.paths.AutoPaths2;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

import java.util.List;

@Autonomous(group = "Close Auto", name = "SOTM Test - Close Start")
public class SOTM_Test extends MyCommandOpMode {
    AutoPaths2 autoPaths2;

    @Override
    public void initialize() {
        r = new MyRobot(
                hardwareMap, telemetry, gamepad1, gamepad2,
                List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET,
                        SubsystemConfig.SHOOTER, SubsystemConfig.DOOR,
                        SubsystemConfig.FOLLOWER),
                OpModeType.AUTO
        );
    }

    @Override
    public void atStart() {
        autoPaths2 = new AutoPaths2(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
        r.overrideAutoEndPose(autoPaths2.getPose(AutoPaths2.PoseId.SINGLE_FRONT_START));

        schedule(
                new SequentialCommandGroup(

                        // Aim turret at goal and spin up shooter before we move
                        new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                        new InstantCommand(() -> r.shooter.setActive(true)),

                        // Drive to mid spike start WHILE shooter tracks via SOTM.
                        // Simultaneously, as soon as the system reports ready, fire.
                        new ParallelDeadlineGroup(
                                // Deadline: path finishes → group ends
                                new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_CLOSE_START_TO_MID_SPIKE_START)),

                                // Side branch: wait until ready then shoot — runs concurrently with path
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() ->
                                                r.shooter.readyToShoot() && r.turret.reachedTarget()
                                        ),
                                        r.shootAll2()
                                )
                        ),

                        // Close door after whatever fired
                        new InstantCommand(() -> r.door.setOpen(false)),

                        // Drive into the spike — done
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_MID_SPIKE_START_TO_MID_SPIKE_END))
                )
        );
    }
}