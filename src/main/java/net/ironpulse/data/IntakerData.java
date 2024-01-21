package net.ironpulse.data;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public record IntakerData(
        Measure<Velocity<Angle>> intakerMotorVelocity
) {
}
