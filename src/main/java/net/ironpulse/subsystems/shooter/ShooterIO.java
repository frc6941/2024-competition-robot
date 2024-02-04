package net.ironpulse.subsystems.shooter;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Amps;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public boolean homed = false;

        public Measure<Velocity<Angle>> leftShooterVelocity = RadiansPerSecond.of(0);
        public Measure<Angle> leftShooterPosition = Radians.of(0);
        public Measure<Voltage> leftShooterAppliedVoltage = Volts.of(0);
        public Measure<Current> leftShooterSupplyCurrent = Amps.of(0);

        public Measure<Velocity<Angle>> rightShooterVelocity = RadiansPerSecond.of(0);
        public Measure<Angle> rightShooterPosition = Radians.of(0);
        public Measure<Voltage> rightShooterAppliedVoltage = Volts.of(0);
        public Measure<Current> rightShooterSupplyCurrent = Amps.of(0);

        public Measure<Angle> armPosition = Radians.of(0);
        public Measure<Voltage> armAppliedVoltage = Volts.of(0);
        public Measure<Current> armSupplyCurrent = Amps.of(0);
    }

    default void updateInputs(ShooterIOInputs inputs) {
    }

    default void setShooterVoltage(Measure<Voltage> volts) {
    }

    default void setArmVoltage(Measure<Voltage> volts) {
    }

    default void setArmHome(Measure<Angle> rad) {
    }

    default void setHomed(boolean homed) {
    }

    default void setArmPosition(Measure<Angle> rad) {
    }
}
