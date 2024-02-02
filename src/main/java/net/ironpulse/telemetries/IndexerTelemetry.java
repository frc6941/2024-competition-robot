package net.ironpulse.telemetries;

import net.ironpulse.data.IndexerData;
import org.littletonrobotics.junction.Logger;

public class IndexerTelemetry {
    public void telemeterize(IndexerData indexerState) {
        Logger.recordOutput("Indexer/Position (degrees)", indexerState.indexerMotorPosition().magnitude());
    }
}
