package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.IndexerSubsystem;
import net.ironpulse.subsystems.ShooterSubsystem;
import net.ironpulse.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class AutoAmpShootCommand extends ParallelCommandGroup {
    public AutoAmpShootCommand(
            RobotContainer robotContainer,
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            SwerveSubsystem swerveSubsystem,
            Supplier<Boolean> confirmation
    ) {
        addCommands(
                new AmpAimingCommand(shooterSubsystem, swerveSubsystem, robotContainer),
                new PreShootCommand(robotContainer, shooterSubsystem),
                Commands.sequence(
                        new WaitUntilCommand(confirmation::get),
                        new DeliverNoteCommand(indexerSubsystem, robotContainer)
                )
        );
    }
}
