package net.ironpulse.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.Getter;
import net.ironpulse.Constants;
import net.ironpulse.data.IntakerData;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static net.ironpulse.Constants.IntakerConstants.INTAKER_MOTOR_ID;

public class IntakerSubsystem implements Subsystem {
    @Getter
    private final TalonFX intakerMotor;

    private final Consumer<IntakerData> telemetryFunction;

    public IntakerSubsystem(Consumer<IntakerData> telemetryFunction) {
        CommandScheduler.getInstance().registerSubsystem(this);
        intakerMotor = new TalonFX(INTAKER_MOTOR_ID,Constants.CAN_BUS_NAME);

        this.telemetryFunction = telemetryFunction;
    }

    @Override
    public void periodic() {
        telemetryFunction.accept(
                new IntakerData(RotationsPerSecond.of(intakerMotor
                        .getVelocity().getValue()))
        );
    }
}
