package net.ironpulse.subsystems.shooter;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public boolean homed = false;

        public Measure<Velocity<Angle>> leftShooterVelocity = RadiansPerSecond.zero();
        public Measure<Angle> leftShooterPosition = Radians.zero();
        public Measure<Voltage> leftShooterAppliedVoltage = Volts.zero();
        public Measure<Current> leftShooterSupplyCurrent = Amps.zero();

        public Measure<Velocity<Angle>> rightShooterVelocity = RadiansPerSecond.zero();
        public Measure<Angle> rightShooterPosition = Radians.zero();
        public Measure<Voltage> rightShooterAppliedVoltage = Volts.zero();
        public Measure<Current> rightShooterSupplyCurrent = Amps.zero();

        public Measure<Angle> armPosition = Radians.zero();
        public Measure<Voltage> armAppliedVoltage = Volts.zero();
        public Measure<Current> armSupplyCurrent = Amps.zero();
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
