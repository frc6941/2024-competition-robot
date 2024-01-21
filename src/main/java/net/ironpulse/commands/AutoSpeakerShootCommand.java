package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.ironpulse.Constants;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.IndexerSubsystem;
import net.ironpulse.subsystems.ShooterSubsystem;
import net.ironpulse.subsystems.SwerveSubsystem;

public class AutoSpeakerShootCommand extends SequentialCommandGroup {
    public AutoSpeakerShootCommand(RobotContainer robotContainer, SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
        addCommands(
                new SpeakerAimingCommand(robotContainer, shooterSubsystem, swerveSubsystem),
                Commands.parallel(
                        new ShootCommand(robotContainer, shooterSubsystem),
                        Commands.sequence(
                                new WaitCommand(Constants.ShooterConstants.shootWaitTime.magnitude()),
                                new DeliverNoteCommand(indexerSubsystem, robotContainer)
                        )
                )
        );
    }
}
