package net.ironpulse.telemetries;

import net.ironpulse.data.IntakerData;
import org.littletonrobotics.junction.Logger;

public class IntakerTelemetry {
    public void telemeterize(IntakerData intakerData) {
        Logger.recordOutput("Intaker/Velocity (rps)", intakerData.intakerMotorVelocity().magnitude());
    }
}
