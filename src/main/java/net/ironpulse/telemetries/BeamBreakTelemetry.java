package net.ironpulse.telemetries;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import net.ironpulse.data.BeamBreakData;

public class BeamBreakTelemetry {
    private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private final NetworkTable beamBreakState = instance.getTable("BeamBreak");

    private final BooleanPublisher fieldIntakerBeamBreak = beamBreakState
            .getBooleanTopic("Is Intaker BeamBreak Triggered").publish();
    private final BooleanPublisher fieldIndexerBeamBreak1 = beamBreakState
            .getBooleanTopic("Is Indexer Beam Break 1 Triggered").publish();

    private final BooleanPublisher fieldIndexerBeamBreak2 = beamBreakState
            .getBooleanTopic("Is Indexer Beam Break 2 Triggered").publish();

    public void telemeterize(BeamBreakData beamBreakData) {
        fieldIntakerBeamBreak.set(beamBreakData.intakerBeamBreak());
        fieldIndexerBeamBreak1.set(beamBreakData.indexerBeamBreak());
        fieldIndexerBeamBreak2.set(beamBreakData.shooterBeamBreak());
    }
}
