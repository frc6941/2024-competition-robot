package net.ironpulse.telemetries;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import net.ironpulse.data.StateData;

public class StateTelemetry {
    private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private final NetworkTable state = instance.getTable("State");

    private final StringPublisher fieldGlobalState = state
            .getStringTopic("Global State").publish();

    public void telemeterize(StateData stateData) {
        fieldGlobalState.set(stateData.currentState().toString());
    }
}
