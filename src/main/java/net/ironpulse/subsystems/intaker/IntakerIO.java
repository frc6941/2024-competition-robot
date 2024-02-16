package net.ironpulse.subsystems.intaker;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface IntakerIO {
    @AutoLog
    class IntakerIOInputs {
        public Measure<Velocity<Angle>> intakeVelocity = RadiansPerSecond.zero();
        public Measure<Angle> intakePosition = Radians.zero();
        public Measure<Voltage> intakeAppliedVoltage = Volts.zero();
        public Measure<Current> intakeSupplyCurrent = Amps.zero();
    }

    default void updateInputs(IntakerIOInputs inputs) {
    }

    default void setIntakeVoltage(Measure<Voltage> volts) {
    }
}
