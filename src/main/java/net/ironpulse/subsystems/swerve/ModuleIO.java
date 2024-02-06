package net.ironpulse.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public Measure<Angle> drivePosition = Radians.of(0);
        public Measure<Velocity<Angle>> driveVelocity = RadiansPerSecond.of(0);
        public Measure<Voltage> driveAppliedVoltage = Volts.of(0);
        public double[] driveCurrentAmps = new double[]{};

        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public Measure<Velocity<Angle>> turnVelocity = RadiansPerSecond.of(0);
        public Measure<Voltage> turnAppliedVoltage = Volts.of(0);
        public double[] turnCurrentAmps = new double[]{};

        public double[] odometryTimestamps = new double[]{};
        public double[] odometryDrivePositionsRad = new double[]{};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[]{};
    }

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(ModuleIOInputs inputs) {
    }

    /**
     * Run the drive motor at the specified voltage.
     */
    default void setDriveVoltage(double volts) {
    }

    /**
     * Run the turn motor at the specified voltage.
     */
    default void setTurnVoltage(double volts) {
    }

    /**
     * Enable or disable brake mode on the drive motor.
     */
    default void setDriveBrakeMode(boolean enable) {
    }

    /**
     * Enable or disable brake mode on the turn motor.
     */
    default void setTurnBrakeMode(boolean enable) {
    }
}