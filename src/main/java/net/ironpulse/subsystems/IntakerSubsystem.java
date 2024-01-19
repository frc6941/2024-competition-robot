package net.ironpulse.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import net.ironpulse.data.IntakerData;
import net.ironpulse.drivers.BeamBreak;

import java.util.function.Consumer;

import static net.ironpulse.Constants.IntakerConstants.BEAM_BREAK_ID;
import static net.ironpulse.Constants.IntakerConstants.INTAKER_MOTOR_ID;

public class IntakerSubsystem {
    private final TalonFX intakerMotor;
    private final BeamBreak beamBreak;

    public IntakerSubsystem() {
        intakerMotor = new TalonFX(INTAKER_MOTOR_ID);
        beamBreak = new BeamBreak(BEAM_BREAK_ID);
    }

    public void registerTelemetry(Consumer<IntakerData> telemetryFunction) {
        telemetryFunction.accept(
                new IntakerData()
        );
    }
}
