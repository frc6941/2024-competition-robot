package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.*;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.IndexerSubsystem;
import net.ironpulse.subsystems.ShooterSubsystem;
import net.ironpulse.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class AutoSpeakerShootCommand extends ParallelCommandGroup {

    public AutoSpeakerShootCommand(
            RobotContainer robotContainer,
            SwerveSubsystem swerveSubsystem,
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            Supplier<Boolean> confirmation
    ) {
        addCommands(
                new SpeakerAimingCommand(robotContainer, shooterSubsystem, swerveSubsystem, confirmation),
                new ShootCommand(robotContainer, shooterSubsystem),
                Commands.sequence(
                        new WaitUntilCommand(confirmation::get),
                        new DeliverNoteCommand(indexerSubsystem, robotContainer)
                )
        );
    }
}
