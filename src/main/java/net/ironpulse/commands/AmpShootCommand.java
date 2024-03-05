package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import java.util.function.BooleanSupplier;

import static net.ironpulse.Constants.ShooterConstants.ampShootingVoltage;

public class AmpShootCommand extends ParallelCommandGroup {
    public AmpShootCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            BooleanSupplier confirmation
    ) {
        addCommands(
                new AmpAimingCommand(shooterSubsystem),
                new PreShootWithoutAimingCommand(shooterSubsystem, ampShootingVoltage),
                Commands.sequence(
                        new WaitUntilCommand(confirmation),
                        new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem)
                )
        );
    }
}
