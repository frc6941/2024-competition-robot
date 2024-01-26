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
    private final BooleanPublisher fieldIndexerBeamBreak = beamBreakState
            .getBooleanTopic("Is Indexer Beam Break Triggered").publish();

    private final BooleanPublisher fieldShooterBeamBreak = beamBreakState
            .getBooleanTopic("Is Shooter Beam Break Triggered").publish();

    public void telemeterize(BeamBreakData beamBreakData) {
        fieldIntakerBeamBreak.set(beamBreakData.intakerBeamBreak());
        fieldIndexerBeamBreak.set(beamBreakData.indexerBeamBreak());
        fieldShooterBeamBreak.set(beamBreakData.shooterBeamBreak());
    }
}
