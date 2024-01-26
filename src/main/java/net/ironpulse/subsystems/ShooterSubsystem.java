package net.ironpulse.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.Getter;
import net.ironpulse.data.ShooterData;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;
import static net.ironpulse.Constants.ShooterConstants.*;

@Getter
public class ShooterSubsystem implements Subsystem {
    private final TalonFX armMotor;
    private final TalonFX shootMotorLeft;
    private final TalonFX shootMotorRight;

    private boolean homed = false;

    private final Consumer<ShooterData> telemetryFunction;

    public ShooterSubsystem(Consumer<ShooterData> telemetryFunction) {
        CommandScheduler.getInstance().registerSubsystem(this);
        this.telemetryFunction = telemetryFunction;
        armMotor = new TalonFX(ARM_MOTOR_ID);
        shootMotorLeft = new TalonFX(SHOOTER_L_MOTOR_ID);
        shootMotorRight = new TalonFX(SHOOTER_R_MOTOR_ID);

        var deployMotorConfig = new TalonFXConfiguration()
                .withSlot0(armGains)
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withFeedback(feedbackConfigs);

        var response = armMotor.getConfigurator().apply(deployMotorConfig);
        if (response.isError())
            System.out.println("Shooter Arm TalonFX failed config with error" + response);
    }

    @Override
    public void periodic() {
        telemetryFunction.accept(
                new ShooterData(
                        Degrees.of(Degrees.convertFrom(armMotor.getPosition()
                                .getValue(), Rotations)),
                        RotationsPerSecond.of(shootMotorLeft.getVelocity().getValue()),
                        RotationsPerSecond.of(shootMotorRight.getVelocity().getValue())
                )
        );
        if (homed) return;
        if (armMotor.getSupplyCurrent().getValue() > armZeroCurrent.magnitude()) {
            armMotor.setPosition(0);
            armMotor.setVoltage(0);
            homed = true;
            return;
        }
        armMotor.setVoltage(armZeroVoltage.magnitude());
    }
}
