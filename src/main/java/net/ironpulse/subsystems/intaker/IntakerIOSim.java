package net.ironpulse.subsystems.intaker;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class IntakerIOSim implements IntakerIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    private final DCMotorSim intakeTalonSim = new DCMotorSim(
            DCMotor.getFalcon500(1),
            6.75, 0.025
    );

    private Measure<Voltage> intakeAppliedVoltage = Volts.zero();

    @Override
    public void updateInputs(IntakerIOInputs inputs) {
        intakeTalonSim.update(LOOP_PERIOD_SECS);

        inputs.intakeVelocity =
                RadiansPerSecond.of(intakeTalonSim.getAngularVelocityRadPerSec());
        inputs.intakePosition =
                Radians.of(MathUtil.inputModulus(intakeTalonSim.getAngularPositionRad(), -2 * Math.PI, 2 * Math.PI));
        inputs.intakeAppliedVoltage =
                intakeAppliedVoltage;
        inputs.intakeSupplyCurrent =
                Amps.of(intakeTalonSim.getCurrentDrawAmps());
    }

    @Override
    public void setIntakeVoltage(Measure<Voltage> volts) {
        intakeAppliedVoltage = volts;
        intakeTalonSim.setInputVoltage(intakeAppliedVoltage.magnitude());
    }
}
