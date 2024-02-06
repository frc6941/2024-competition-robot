package net.ironpulse.subsystems.intaker;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface IntakerIO {
    @AutoLog
    class IntakerIOInputs {
        public Measure<Velocity<Angle>> intakeVelocity = RadiansPerSecond.of(0);
        public Measure<Angle> intakePosition = Radians.of(0);
        public Measure<Voltage> intakeAppliedVoltage = Volts.of(0);
        public Measure<Current> intakeSupplyCurrent = Amps.of(0);
    }

    default void updateInputs(IntakerIOInputs inputs) {
    }

    default void setIntakeVoltage(Measure<Voltage> volts) {
    }
}
