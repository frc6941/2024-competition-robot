package net.ironpulse.subsystems.beambreak;

import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import net.ironpulse.Constants;
import net.ironpulse.drivers.BeamBreak;

public class BeamBreakIOSim implements BeamBreakIO {
    private final AnalogInputSim intakerSim =
            new AnalogInputSim(Constants.BeamBreakConstants.INTAKER_BEAM_BREAK_ID);
    private final AnalogInputSim indexerSim =
            new AnalogInputSim(Constants.BeamBreakConstants.INDEXER_BEAM_BREAK_ID);
    private final AnalogInputSim shooterSim =
            new AnalogInputSim(Constants.BeamBreakConstants.SHOOTER_BEAM_BREAK_ID);
    private final BeamBreak intakerBeamBreak =
            new BeamBreak(Constants.BeamBreakConstants.INTAKER_BEAM_BREAK_ID);
    private final BeamBreak indexerBeamBreak =
            new BeamBreak(Constants.BeamBreakConstants.INDEXER_BEAM_BREAK_ID);
    private final BeamBreak shooterBeamBreak =
            new BeamBreak(Constants.BeamBreakConstants.SHOOTER_BEAM_BREAK_ID);

    public BeamBreakIOSim() {
        // do not fix it
        intakerSim.setInitialized(true);
        indexerSim.setInitialized(true);
        shooterSim.setInitialized(true);
    }

    @Override
    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.isIntakerBeamBreakOn = intakerBeamBreak.get();
        inputs.isIndexerBeamBreakOn = indexerBeamBreak.get();
        inputs.isShooterBeamBreakOn = shooterBeamBreak.get();
    }
}
