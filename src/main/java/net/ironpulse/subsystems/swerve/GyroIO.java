package net.ironpulse.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double[] odometryYawTimestamps = new double[]{};
        public Rotation2d[] odometryYawPositions = new Rotation2d[]{};
        public Measure<Velocity<Angle>> yawVelocity =
                RadiansPerSecond.zero();
    }

    default void zeroGyro() {
    }

    void updateInputs(GyroIOInputs inputs);
}
