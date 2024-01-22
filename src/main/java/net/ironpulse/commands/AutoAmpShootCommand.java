package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.ironpulse.Constants;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.IndexerSubsystem;
import net.ironpulse.subsystems.ShooterSubsystem;

public class AutoAmpShootCommand extends SequentialCommandGroup {
    public AutoAmpShootCommand(
            RobotContainer robotContainer,
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem
    ) {
        addCommands(
                new AmpAimingCommand(shooterSubsystem, robotContainer),
                Commands.parallel(
                        new PreShootCommand(robotContainer, shooterSubsystem),
                        Commands.sequence(
                                new WaitCommand(Constants.ShooterConstants.shootWaitTime.magnitude()),
                                new DeliverNoteCommand(indexerSubsystem, robotContainer)
                        )
                )
        );
    }
}
