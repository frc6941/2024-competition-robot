package net.ironpulse.commands.autos;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

public class AutoShootWithAngleCommand extends ParallelCommandGroup {
    public AutoShootWithAngleCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            int angle
    ) {
        Measure<Angle> deployAngle = Degrees.of(angle);
        addCommands(
                new AutoAimingWithAngleCommand(shooterSubsystem, deployAngle),
                Commands.sequence(
                        new WaitCommand(0.5),
                        new AutoDeliverNoteCommand(indexerSubsystem)
                )
        );
    }
}
