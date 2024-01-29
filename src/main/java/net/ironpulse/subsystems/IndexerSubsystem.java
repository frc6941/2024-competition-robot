package net.ironpulse.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.Getter;
import net.ironpulse.Constants;
import net.ironpulse.data.IndexerData;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;
import static net.ironpulse.Constants.IndexerConstants.*;

public class IndexerSubsystem implements Subsystem {
    @Getter
    private final TalonFX indexerMotor;

    private final Consumer<IndexerData> telemetryFunction;

    public IndexerSubsystem(Consumer<IndexerData> telemetryFunction) {
        CommandScheduler.getInstance().registerSubsystem(this);
        indexerMotor = new TalonFX(INDEXER_MOTOR_ID,
                Constants.CAN_BUS_NAME);
        var indexerMotorConfigs = new TalonFXConfiguration()
                .withMotorOutput(motorOutputConfigs);
        var response = indexerMotor.getConfigurator().apply(indexerMotorConfigs);
        if (response.isError())
            System.out.println("Indexer TalonFX failed config with error" + response);

        this.telemetryFunction = telemetryFunction;
    }

    @Override
    public void periodic() {
        telemetryFunction.accept(
                new IndexerData(
                        Degrees.of(Degrees.convertFrom(
                                indexerMotor.getPosition().getValue(), Rotations))
                )
        );
    }
}
