package net.ironpulse.telemetries;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import net.ironpulse.data.IndexerData;

public class IndexerTelemetry {
    private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private final NetworkTable indexerState = instance.getTable("Indexer");

    private final DoublePublisher fieldIndexerPosition = indexerState
            .getDoubleTopic("Position (degrees)").publish();

    public void telemeterize(IndexerData indexerState) {
        fieldIndexerPosition.set(indexerState.indexerMotorPosition().magnitude());
    }
}
