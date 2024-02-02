package net.ironpulse.telemetries;

import net.ironpulse.data.BeamBreakData;
import org.littletonrobotics.junction.Logger;

public class BeamBreakTelemetry {

    public void telemeterize(BeamBreakData beamBreakData) {
        Logger.recordOutput("BeamBreak/Intaker BeamBreak Triggered", beamBreakData.intakerBeamBreak());
        Logger.recordOutput("BeamBreak/Indexer BeamBreak Triggered", beamBreakData.indexerBeamBreak());
        Logger.recordOutput("BeamBreak/Shooter BeamBreak Triggered", beamBreakData.shooterBeamBreak());
    }
}
