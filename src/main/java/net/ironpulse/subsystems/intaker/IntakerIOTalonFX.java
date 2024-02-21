package net.ironpulse.subsystems.intaker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import net.ironpulse.Constants;

import static edu.wpi.first.units.Units.*;
import static net.ironpulse.Constants.IndexerConstants.motorOutputConfigs;
import static net.ironpulse.Constants.IntakerConstants.INTAKE_MOTOR_ID;

public class IntakerIOTalonFX implements IntakerIO {
    private final TalonFX intakeTalon = new TalonFX(INTAKE_MOTOR_ID, Constants.CAN_BUS_NAME);

    private final StatusSignal<Double> intakeVelocity = intakeTalon.getVelocity();
    private final StatusSignal<Double> intakePosition = intakeTalon.getPosition();
    private final StatusSignal<Double> intakeAppliedVoltage = intakeTalon.getMotorVoltage();
    private final StatusSignal<Double> intakeSupplyCurrent = intakeTalon.getSupplyCurrent();

    public IntakerIOTalonFX() {
        var intakerMotorConfigs = new TalonFXConfiguration()
                .withMotorOutput(motorOutputConfigs);
        var response = intakeTalon.getConfigurator().apply(intakerMotorConfigs);
        if (response.isError())
            System.out.println("Intaker TalonFX failed config with error" + response);
        response = intakeTalon.clearStickyFaults();
        if (response.isError())
            System.out.println("Intaker TalonFX failed sticky fault clearing with error" + response);
    }

    @Override
    public void updateInputs(IntakerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                intakeVelocity,
                intakePosition,
                intakeAppliedVoltage,
                intakeSupplyCurrent
        );

        inputs.intakeVelocity =
                RadiansPerSecond.of(Units.rotationsToRadians(intakeVelocity.getValueAsDouble()));
        inputs.intakePosition =
                Radians.of(Units.rotationsToRadians(intakePosition.getValueAsDouble()));
        inputs.intakeAppliedVoltage =
                Volts.of(intakeAppliedVoltage.getValueAsDouble());
        inputs.intakeSupplyCurrent =
                Amps.of(intakeSupplyCurrent.getValueAsDouble());
    }

    @Override
    public void setIntakeVoltage(Measure<Voltage> volts) {
        intakeTalon.setControl(new VoltageOut(volts.magnitude()));
    }
}
