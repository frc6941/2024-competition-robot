package net.ironpulse.commands;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static net.ironpulse.Constants.ShooterConstants.farShootVoltage;

public class ParallelShootWithDelayCommand extends ParallelCommandGroup {
    public ParallelShootWithDelayCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            Measure<Angle> angle
    ) {
        addCommands(
                new ParallelAimingCommand(shooterSubsystem, angle),
                new PreShootWithoutAimingCommand(shooterSubsystem, farShootVoltage),
                Commands.sequence(
                        new WaitCommand(0.5),
                        new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem)
                )
        );
    }
}