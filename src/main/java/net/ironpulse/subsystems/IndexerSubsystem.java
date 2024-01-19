package net.ironpulse.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import net.ironpulse.Constants;
import net.ironpulse.data.IndexerData;
import net.ironpulse.drivers.BeamBreak;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;
import static net.ironpulse.Constants.IndexerConstants.*;

public class IndexerSubsystem implements Subsystem {
    private final TalonFX indexerMotor;
    private final BeamBreak beamBreak;

    public IndexerSubsystem() {
        this.indexerMotor = new TalonFX(Constants.IndexerConstants.INDEXER_MOTOR_ID);
        TalonFXConfiguration indexerMotorConfig = new TalonFXConfiguration()
                .withSlot0(indexerGains)
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withFeedback(feedbackConfigs);

        var response = indexerMotor.getConfigurator().apply(indexerMotorConfig);
        if (response.isError())
            System.out.println("Indexer TalonFX failed config with error" + response);

        beamBreak = new BeamBreak(BEAM_BREAK_ID);
    }

    public void registerTelemetry(Consumer<IndexerData> telemetryFunction) {
        telemetryFunction.accept(
                new IndexerData(
                        RotationsPerSecond.of(indexerMotor.getVelocity().getValue()),
                        Degrees.of(Degrees.convertFrom(
                                indexerMotor.getPosition().getValue(), Rotations))
                )
        );
    }
}
