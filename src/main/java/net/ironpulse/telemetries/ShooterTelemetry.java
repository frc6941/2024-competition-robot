package net.ironpulse.telemetries;

import net.ironpulse.data.ShooterData;
import org.littletonrobotics.junction.Logger;

public class ShooterTelemetry {
    public void telemeterize(ShooterData shooterData) {
        Logger.recordOutput("Shooter/Velocity L (rps)", shooterData.shootMotorLeftVelocity().magnitude());
        Logger.recordOutput("Shooter/Velocity R (rps)", shooterData.shootMotorRightVelocity().magnitude());
        Logger.recordOutput("Shooter/Position (degrees)", shooterData.deployMotorPosition().magnitude());
        Logger.recordOutput("Shooter/Supply Current (amps)", shooterData.armMotorSupplyCurrent().magnitude());
    }
}
