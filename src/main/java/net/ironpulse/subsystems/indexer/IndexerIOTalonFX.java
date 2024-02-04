package net.ironpulse.subsystems.indexer;

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
import static net.ironpulse.Constants.IndexerConstants.INDEX_MOTOR_ID;
import static net.ironpulse.Constants.IndexerConstants.motorOutputConfigs;

public class IndexerIOTalonFX implements IndexerIO {
    private final TalonFX indexTalon = new TalonFX(INDEX_MOTOR_ID, Constants.CAN_BUS_NAME);

    private final StatusSignal<Double> indexVelocity = indexTalon.getVelocity();
    private final StatusSignal<Double> indexPosition = indexTalon.getPosition();
    private final StatusSignal<Double> indexAppliedVoltage = indexTalon.getMotorVoltage();
    private final StatusSignal<Double> indexSupplyCurrent = indexTalon.getSupplyCurrent();

    public IndexerIOTalonFX() {
        var indexerMotorConfigs = new TalonFXConfiguration()
                .withMotorOutput(motorOutputConfigs);
        var response = indexTalon.getConfigurator().apply(indexerMotorConfigs);
        if (response.isError())
            System.out.println("Indexer TalonFX failed config with error" + response);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                indexVelocity,
                indexPosition,
                indexAppliedVoltage,
                indexSupplyCurrent
        );

        inputs.indexVelocity =
                RadiansPerSecond.of(Units.rotationsToRadians(indexVelocity.getValueAsDouble()));
        inputs.indexPosition =
                Radians.of(Units.rotationsToRadians(indexPosition.getValueAsDouble()));
        inputs.indexAppliedVoltage =
                Volts.of(indexAppliedVoltage.getValueAsDouble());
        inputs.indexSupplyCurrent =
                Amps.of(indexSupplyCurrent.getValueAsDouble());
    }

    @Override
    public void setIndexVoltage(Measure<Voltage> volts) {
        indexTalon.setControl(new VoltageOut(volts.magnitude()));
    }
}
