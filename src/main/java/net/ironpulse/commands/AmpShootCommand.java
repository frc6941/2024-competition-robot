package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.IndexerSubsystem;
import net.ironpulse.subsystems.ShooterSubsystem;

import java.util.function.Supplier;

/**
 * This command will first aim the Amp and at the same time set the shooter to a voltage.
 * When the command get confirmation from driver, the indexer will deliver the note into
 * the shooter and finish a shot.
 * <p>
 * End Condition: Once robot state transferred from SHOOTING to IDLE
 */
public class AmpShootCommand extends ParallelCommandGroup {
    public AmpShootCommand(
            RobotContainer robotContainer,
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            Supplier<Boolean> confirmation
    ) {
        addCommands(
                new AmpAimingCommand(shooterSubsystem, robotContainer),
                new PreShootCommand(robotContainer, shooterSubsystem),
                Commands.sequence(
                        new WaitUntilCommand(confirmation::get),
                        new DeliverNoteCommand(indexerSubsystem, robotContainer)
                )
        );
    }
}
