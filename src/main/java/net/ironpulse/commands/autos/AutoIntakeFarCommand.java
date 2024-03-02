package net.ironpulse.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.intaker.IntakerSubsystem;

import static edu.wpi.first.units.Units.Volts;
import static net.ironpulse.Constants.IndexerConstants.indexVoltage;
import static net.ironpulse.Constants.IntakerConstants.intakeVoltage;
import static net.ironpulse.Constants.Logger.debug;
import static net.ironpulse.utils.Utils.autoIntaking;

public class AutoIntakeFarCommand extends Command {
    private final IntakerSubsystem intakerSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;

    private final Timer timer = new Timer();

    public AutoIntakeFarCommand(
            IntakerSubsystem intakerSubsystem,
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem
    ) {
        this.intakerSubsystem = intakerSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
    }

    @Override
    public void initialize() {
        debug("AutoIntakeFr", "start");
        timer.restart();
        autoIntaking = true;
    }

    @Override
    public void execute() {
        intakerSubsystem.getIo().setIntakeVoltage(intakeVoltage);
        indexerSubsystem.getIo().setIndexVoltage(indexVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        debug("AutoIntakeFar", "end; elapsed=" + timer.get());
        intakerSubsystem.getIo().setIntakeVoltage(Volts.zero());
        indexerSubsystem.getIo().setIndexVoltage(Volts.zero());
        autoIntaking = false;
    }

    @Override
    public boolean isFinished() {
        return beamBreakSubsystem.getInputs().isIndexerBeamBreakOn &&
                !beamBreakSubsystem.getInputs().isIntakerBeamBreakOn ||
                timer.hasElapsed(3.5);
    }
}