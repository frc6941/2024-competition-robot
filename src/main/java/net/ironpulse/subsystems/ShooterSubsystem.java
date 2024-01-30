package net.ironpulse.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.Getter;
import lombok.Setter;
import net.ironpulse.Constants;
import net.ironpulse.data.ShooterData;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;
import static net.ironpulse.Constants.ShooterConstants.*;

@Getter
public class ShooterSubsystem implements Subsystem {
    private final TalonFX armMotor;
    private final TalonFX shootMotorLeft;
    private final TalonFX shootMotorRight;

    @Setter
    private boolean homed = false;

    private final Consumer<ShooterData> telemetryFunction;

    public ShooterSubsystem(Consumer<ShooterData> telemetryFunction) {
        CommandScheduler.getInstance().registerSubsystem(this);
        this.telemetryFunction = telemetryFunction;
        armMotor = new TalonFX(ARM_MOTOR_ID, Constants.CAN_BUS_NAME);
        shootMotorLeft = new TalonFX(SHOOTER_L_MOTOR_ID, Constants.CAN_BUS_NAME);
        shootMotorRight = new TalonFX(SHOOTER_R_MOTOR_ID, Constants.CAN_BUS_NAME);

        var armMotorConfig = new TalonFXConfiguration()
                .withSlot0(armGainsUp)
                .withSlot1(armGainsDown)
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withFeedback(feedbackConfigs);

        var response = armMotor.getConfigurator().apply(armMotorConfig);
        if (response.isError())
            System.out.println("Shooter Arm TalonFX failed config with error" + response);
    }

    @Override
    public void periodic() {
        telemetryFunction.accept(
                new ShooterData(
                        Degrees.of(Rotations.of(armMotor.getPosition()
                                .getValue()).in(Degrees)),
                        RotationsPerSecond.of(shootMotorLeft.getVelocity().getValue()),
                        RotationsPerSecond.of(shootMotorRight.getVelocity().getValue()),
                        Amps.of(armMotor.getSupplyCurrent().getValue())
                )
        );
        if (homed) return;
        if (armMotor.getSupplyCurrent().getValue() > armZeroCurrent.magnitude()) {
            armMotor.setControl(new VoltageOut(0));
            armMotor.setPosition(0);
            homed = true;
            return;
        }
        armMotor.setControl(new VoltageOut(armZeroVoltage.magnitude()));
    }
}
