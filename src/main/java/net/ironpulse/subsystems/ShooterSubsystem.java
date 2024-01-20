package net.ironpulse.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import net.ironpulse.data.ShooterData;

import java.util.function.Consumer;

import static net.ironpulse.Constants.ShooterConstants.*;

public class ShooterSubsystem implements Subsystem {
    private final TalonFX deployMotor;
    private final TalonFX shootMotor;

    private final Consumer<ShooterData> telemetryFunction;

    public ShooterSubsystem(Consumer<ShooterData> telemetryFunction) {
        this.telemetryFunction = telemetryFunction;
        deployMotor = new TalonFX(DEPLOY_MOTOR_ID);
        shootMotor = new TalonFX(SHOOT_MOTOR_ID);

        var deployMotorConfig = new TalonFXConfiguration()
                .withSlot0(deployGains)
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withFeedback(feedbackConfigs);

        var response = deployMotor.getConfigurator().apply(deployMotorConfig);
        if (response.isError())
            System.out.println("Shooter Deploy TalonFX failed config with error" + response);
    }

    @Override
    public void periodic() {
        telemetryFunction.accept(
                new ShooterData()
        );
    }
}
