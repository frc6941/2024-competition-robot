package net.ironpulse.telemetries;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import net.ironpulse.data.IntakerData;

public class IntakerTelemetry {
    private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private final NetworkTable intakerState = instance.getTable("Intaker");

    private final DoublePublisher fieldIntakerVelocity = intakerState
            .getDoubleTopic("Velocity (r/s)").publish();

    public void telemeterize(IntakerData intakerData) {
        fieldIntakerVelocity.set(intakerData.intakerMotorVelocity().magnitude());
    }
}
