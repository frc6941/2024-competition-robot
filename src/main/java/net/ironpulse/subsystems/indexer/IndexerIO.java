package net.ironpulse.subsystems.indexer;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public Measure<Velocity<Angle>> indexVelocity = RadiansPerSecond.of(0);
        public Measure<Angle> indexPosition = Radians.of(0);
        public Measure<Voltage> indexAppliedVoltage = Volts.of(0);
        public Measure<Current> indexSupplyCurrent = Amps.of(0);
    }

    default void updateInputs(IndexerIOInputs inputs) {
    }

    default void setIndexVoltage(Measure<Voltage> volts) {
    }
}
