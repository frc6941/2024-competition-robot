package net.ironpulse.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class IndexerIOSim implements IndexerIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    private final DCMotorSim indexTalonSim = new DCMotorSim(
            DCMotor.getFalcon500Foc(1), 6.75, 0.025
    );

    private Measure<Voltage> indexAppliedVoltage = Volts.of(0);

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        indexTalonSim.update(LOOP_PERIOD_SECS);

        inputs.indexVelocity =
                RadiansPerSecond.of(indexTalonSim.getAngularVelocityRadPerSec());
        inputs.indexPosition =
                Radians.of(MathUtil.inputModulus(indexTalonSim.getAngularPositionRad(), -2 * Math.PI, 2 * Math.PI));
        inputs.indexAppliedVoltage =
                indexAppliedVoltage;
        inputs.indexSupplyCurrent =
                Amps.of(indexTalonSim.getCurrentDrawAmps());
    }

    @Override
    public void setIndexVoltage(Measure<Voltage> volts) {
        indexAppliedVoltage = volts;
        indexTalonSim.setInputVoltage(indexAppliedVoltage.magnitude());
    }
}
