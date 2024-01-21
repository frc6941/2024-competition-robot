package net.ironpulse.telemetries;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import net.ironpulse.data.ShooterData;

public class ShooterTelemetry {
    private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private final NetworkTable shooterState = instance.getTable("Shooter");

    private final DoublePublisher fieldShooterVelocity = shooterState
            .getDoubleTopic("Velocity (r/s)").publish();
    private final DoublePublisher fieldShooterPosition = shooterState
            .getDoubleTopic("Position (degrees)").publish();

    public void telemeterize(ShooterData shooterData) {
        fieldShooterVelocity.set(shooterData.shootMotorVelocity().magnitude());
        fieldShooterPosition.set(shooterData.deployMotorPosition().magnitude());
    }
}
