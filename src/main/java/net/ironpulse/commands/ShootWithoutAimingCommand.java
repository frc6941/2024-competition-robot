package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import java.util.function.Supplier;

import static net.ironpulse.Constants.ShooterConstants.farShootVoltage;

public class ShootWithoutAimingCommand extends ParallelCommandGroup {
    public ShootWithoutAimingCommand(
            IndicatorSubsystem indicatorSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            Supplier<Boolean> confirmation
    ) {
        addCommands(
                new PreShootWithoutAimingCommand(shooterSubsystem, farShootVoltage),
                Commands.sequence(
                        new WaitUntilCommand(confirmation::get),
                        new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem)
                )
        );
    }
}
