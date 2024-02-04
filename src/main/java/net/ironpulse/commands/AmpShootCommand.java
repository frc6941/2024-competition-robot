package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import java.util.function.BooleanSupplier;

public class AmpShootCommand extends ParallelCommandGroup {
    public AmpShootCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            RobotContainer robotContainer,
            BooleanSupplier confirmation
    ) {
        addCommands(
                new AmpAimingCommand(shooterSubsystem),
                new PreShootCommand(shooterSubsystem, robotContainer),
                Commands.sequence(
                        new WaitUntilCommand(confirmation),
                        new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, robotContainer)
                )
        );
    }
}
