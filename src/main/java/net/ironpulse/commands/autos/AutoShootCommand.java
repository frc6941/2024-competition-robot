package net.ironpulse.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;

public class AutoShootCommand extends SequentialCommandGroup {
    public AutoShootCommand(
            IndexerSubsystem indexerSubsystem
    ) {
        addCommands(
                new WaitCommand(0.75),
                new AutoDeliverNoteCommand(indexerSubsystem)
        );
    }
}
