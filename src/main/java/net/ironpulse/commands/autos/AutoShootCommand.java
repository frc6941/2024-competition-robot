package net.ironpulse.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.ironpulse.subsystems.IndexerSubsystem;
import net.ironpulse.subsystems.ShooterSubsystem;
import net.ironpulse.subsystems.drive.Drive;

public class AutoShootCommand extends ParallelCommandGroup {
    public AutoShootCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            Drive swerveSubsystem
    ) {
        addCommands(
                new AutoAimingCommand(shooterSubsystem, swerveSubsystem),
                new AutoPreShootCommand(shooterSubsystem),
                Commands.sequence(
                        new WaitCommand(1.5),
                        new AutoDeliverNoteCommand(indexerSubsystem)
                )
        );
    }
}
