package net.ironpulse.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;

public class AutoShootCommand extends SequentialCommandGroup {
    public AutoShootCommand(
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem
    ) {
        var noNotesInside =
                !beamBreakSubsystem.getInputs().isIndexerBeamBreakOn &&
                        !beamBreakSubsystem.getInputs().isShooterBeamBreakOn &&
                        !beamBreakSubsystem.getInputs().isIntakerBeamBreakOn;
        addCommands(
                new WaitCommand(0.75).onlyIf(() -> !noNotesInside),
                new AutoDeliverNoteCommand(indexerSubsystem, beamBreakSubsystem)
        );
    }
}
