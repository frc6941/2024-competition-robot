package net.ironpulse.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;

import static edu.wpi.first.units.Units.Volts;
import static net.ironpulse.Constants.IndexerConstants.indexVoltage;
import static net.ironpulse.Constants.Logger.debug;

public class AutoDeliverNoteCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;

    private final Timer timer = new Timer();

    public AutoDeliverNoteCommand(IndexerSubsystem indexerSubsystem, BeamBreakSubsystem beamBreakSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
    }

    @Override
    public void initialize() {
        debug("AutoDeliverNote", "start");
        timer.restart();
    }

    @Override
    public void execute() {
        indexerSubsystem.getIo().setIndexVoltage(indexVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        debug("AutoDeliverNote", "end; elapsed=" + timer.get());
        indexerSubsystem.getIo().setIndexVoltage(Volts.zero());
    }

    @Override
    public boolean isFinished() {
        // fail fast when no notes inside
        var noNotesInside =
                !beamBreakSubsystem.getInputs().isIndexerBeamBreakOn &&
                        !beamBreakSubsystem.getInputs().isShooterBeamBreakOn &&
                        !beamBreakSubsystem.getInputs().isIntakerBeamBreakOn;
        return timer.hasElapsed(0.2) || noNotesInside;
    }
}