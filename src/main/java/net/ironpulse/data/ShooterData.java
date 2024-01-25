package net.ironpulse.data;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public record ShooterData(
        Measure<Angle> deployMotorPosition,
        Measure<Velocity<Angle>> shootMotorVelocity
) {
}
