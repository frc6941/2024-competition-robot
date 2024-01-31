package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.*;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.IndexerSubsystem;
import net.ironpulse.subsystems.ShooterSubsystem;
import net.ironpulse.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class SpeakerShootCommand extends ParallelCommandGroup {

    public SpeakerShootCommand(
            RobotContainer robotContainer,
            SwerveSubsystem swerveSubsystem,
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            Supplier<Boolean> confirmation
    ) {
        addCommands(
                new SpeakerAimingCommand(robotContainer, shooterSubsystem, swerveSubsystem),
                new PreShootCommand(shooterSubsystem, robotContainer),
                Commands.sequence(
                        new WaitUntilCommand(confirmation::get),
                        new DeliverNoteCommand(indexerSubsystem, robotContainer)
                )
        );
    }
}
