package net.ironpulse.subsystems.indexer;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public Measure<Velocity<Angle>> indexVelocity = RadiansPerSecond.zero();
        public Measure<Angle> indexPosition = Radians.zero();
        public Measure<Voltage> indexAppliedVoltage = Volts.zero();
        public Measure<Current> indexSupplyCurrent = Amps.zero();
    }

    void updateInputs(IndexerIOInputs inputs);

    void setIndexVoltage(Measure<Voltage> volts);
}
