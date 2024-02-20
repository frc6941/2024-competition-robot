package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

public class ShootPlateCommand extends ParallelCommandGroup {
    public ShootPlateCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            BooleanSupplier confirmation
    ) {
        addCommands(
                Commands.sequence(
                        new ShooterUpCommand(shooterSubsystem).onlyWhile(
                                () -> Rotations.of(shooterSubsystem.getInputs().armPosition.magnitude()).in(Degrees) < 90
                        ),
                        new PreShootIndexCommand(indexerSubsystem),
                        new WaitUntilCommand(confirmation),
                        new DeliverNoteIndexCommand(shooterSubsystem, beamBreakSubsystem)
                )
        );
    }
}
