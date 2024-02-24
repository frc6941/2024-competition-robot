package net.ironpulse.commands;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import java.util.function.BooleanSupplier;

import static net.ironpulse.Constants.ShooterConstants.defaultShootVoltage;

public class ParallelShootCommand extends ParallelCommandGroup {
    public ParallelShootCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            BooleanSupplier confirmation,
            Measure<Angle> angle
    ) {
        addCommands(
                new ParallelAimingCommand(shooterSubsystem, angle),
                new PreShootWithoutAimingCommand(shooterSubsystem, defaultShootVoltage),
                Commands.sequence(
                        new WaitUntilCommand(confirmation),
                        new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem)
                )
        );
    }
}