package net.ironpulse.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

public class AutoShootCommand extends ParallelCommandGroup {
    public AutoShootCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem
    ) {
        addCommands(
                new AutoAimingCommand(shooterSubsystem),
                Commands.sequence(
//                        new WaitCommand(0.75), // FIXME
                        new AutoDeliverNoteCommand(indexerSubsystem)
                )
        );
    }
}