package net.ironpulse.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import net.ironpulse.Constants;

import static edu.wpi.first.units.Units.*;
import static net.ironpulse.Constants.ShooterConstants.*;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX leftShooterTalon = new TalonFX(LEFT_SHOOTER_MOTOR_ID, Constants.CAN_BUS_NAME);
    private final TalonFX rightShooterTalon = new TalonFX(RIGHT_SHOOTER_MOTOR_ID, Constants.CAN_BUS_NAME);
    private final TalonFX armTalon = new TalonFX(ARM_MOTOR_ID, Constants.CAN_BUS_NAME);
    private final TalonFX pullerTalon = new TalonFX(PULLER_MOTOR_ID, Constants.CAN_BUS_NAME);

    private boolean homed = false;

    private final StatusSignal<Double> leftShooterVelocity = leftShooterTalon.getVelocity();
    private final StatusSignal<Double> leftShooterPosition = leftShooterTalon.getPosition();
    private final StatusSignal<Double> leftShooterAppliedVoltage = leftShooterTalon.getMotorVoltage();
    private final StatusSignal<Double> leftShooterSupplyCurrent = leftShooterTalon.getSupplyCurrent();

    private final StatusSignal<Double> rightShooterVelocity = rightShooterTalon.getVelocity();
    private final StatusSignal<Double> rightShooterPosition = rightShooterTalon.getPosition();
    private final StatusSignal<Double> rightShooterAppliedVoltage = rightShooterTalon.getMotorVoltage();
    private final StatusSignal<Double> rightShooterSupplyCurrent = rightShooterTalon.getSupplyCurrent();

    private final StatusSignal<Double> armPosition = armTalon.getPosition();
    private final StatusSignal<Double> armAppliedVoltage = armTalon.getMotorVoltage();
    private final StatusSignal<Double> armSupplyCurrent = armTalon.getSupplyCurrent();
    private final StatusSignal<Double> armTorqueCurrent = armTalon.getTorqueCurrent();

    private final StatusSignal<Double> pullerPosition = pullerTalon.getPosition();
    private final StatusSignal<Double> pullerAppliedVoltage = pullerTalon.getMotorVoltage();
    private final StatusSignal<Double> pullerSupplyCurrent = pullerTalon.getSupplyCurrent();
    private final StatusSignal<Double> pullerTorqueCurrent = pullerTalon.getTorqueCurrent();

    public ShooterIOTalonFX() {
        var armMotorConfig = new TalonFXConfiguration()
                .withSlot0(armGainsUp)
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withFeedback(feedbackConfigs);
        var response = armTalon.getConfigurator().apply(armMotorConfig);
        if (response.isError())
            System.out.println("Shooter Arm TalonFX failed config with error" + response);
        armTalon.setPosition(0);
        var pullerMotorConfig = new TalonFXConfiguration()
                .withFeedback(pullerfeedbackConfigs);
        pullerTalon.setPosition(0);
        response = pullerTalon.getConfigurator().apply(pullerMotorConfig);
        if (response.isError())
            System.out.println("Puller TalonFX failed config with error" + response);
        response = armTalon.clearStickyFaults();
        if (response.isError())
            System.out.println("Shooter Arm TalonFX failed sticky fault clearing with error" + response);
        response = pullerTalon.clearStickyFaults();
        if (response.isError())
            System.out.println("Puller TalonFX failed sticky fault clearing with error" + response);
        response = leftShooterTalon.clearStickyFaults();
        if (response.isError())
            System.out.println("Left Shooter TalonFX failed sticky fault clearing with error" + response);
        response = rightShooterTalon.clearStickyFaults();
        if (response.isError())
            System.out.println("Right Shooter TalonFX failed sticky fault clearing with error" + response);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftShooterVelocity,
                leftShooterPosition,
                leftShooterAppliedVoltage,
                leftShooterSupplyCurrent,
                rightShooterVelocity,
                rightShooterPosition,
                rightShooterAppliedVoltage,
                rightShooterSupplyCurrent,
                armPosition,
                armAppliedVoltage,
                armSupplyCurrent,
                armTorqueCurrent,
                pullerPosition,
                pullerAppliedVoltage,
                pullerSupplyCurrent,
                pullerTorqueCurrent
        );

        inputs.leftShooterVelocity =
                RadiansPerSecond.of(Units.rotationsToRadians(leftShooterVelocity.getValueAsDouble()));
        inputs.leftShooterPosition =
                Radians.of(Units.rotationsToRadians(leftShooterPosition.getValueAsDouble()));
        inputs.leftShooterAppliedVoltage =
                Volts.of(leftShooterAppliedVoltage.getValueAsDouble());
        inputs.leftShooterSupplyCurrent =
                Amps.of(leftShooterSupplyCurrent.getValueAsDouble());

        inputs.rightShooterVelocity =
                RadiansPerSecond.of(Units.rotationsToRadians(rightShooterVelocity.getValueAsDouble()));
        inputs.rightShooterPosition =
                Radians.of(Units.rotationsToRadians(rightShooterPosition.getValueAsDouble()));
        inputs.rightShooterAppliedVoltage =
                Volts.of(rightShooterAppliedVoltage.getValueAsDouble());
        inputs.rightShooterSupplyCurrent =
                Amps.of(rightShooterSupplyCurrent.getValueAsDouble());

        inputs.armPosition =
                Radians.of(Units.rotationsToRadians(armPosition.getValueAsDouble()));
        inputs.armAppliedVoltage =
                Volts.of(armAppliedVoltage.getValueAsDouble());
        inputs.armSupplyCurrent =
                Amps.of(armSupplyCurrent.getValueAsDouble());
        inputs.armTorqueCurrent =
                Amps.of(armTorqueCurrent.getValueAsDouble());

        inputs.pullerPosition =
                Radians.of(Units.rotationsToRadians(pullerPosition.getValueAsDouble()));
        inputs.pullerAppliedVoltage =
                Volts.of(pullerAppliedVoltage.getValueAsDouble());
        inputs.pullerSupplyCurrent =
                Amps.of(pullerSupplyCurrent.getValueAsDouble());
        inputs.pullerTorqueCurrent =
                Amps.of(pullerTorqueCurrent.getValueAsDouble());

        inputs.homed = homed;
    }

    @Override
    public void setShooterVoltage(Measure<Voltage> volts) {
        leftShooterTalon.setControl(new VoltageOut(volts.magnitude()));
        rightShooterTalon.setControl(new Follower(leftShooterTalon.getDeviceID(),
                true));
    }

    @Override
    public void setArmVoltage(Measure<Voltage> volts) {
        armTalon.setControl(new VoltageOut(volts.magnitude()));
    }

    @Override
    public void setPullerVoltage(Measure<Voltage> volts) {
        pullerTalon.setControl(new VoltageOut(volts.magnitude()));
    }

    @Override
    public void setArmHome(Measure<Angle> rad) {
        armTalon.setPosition(rad.in(Rotations));
    }

    @Override
    public void setArmPosition(Measure<Angle> rad) {
        armTalon.setControl(new MotionMagicVoltage(rad.in(Rotations)));
    }

    @Override
    public void setArmBrakeMode(boolean isCoast) {
        var config = new MotorOutputConfigs();
        config.NeutralMode = isCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        armTalon.getConfigurator().apply(config);
        armTalon.setControl(new NeutralOut());
    }

    @Override
    public void setPullerBrakeMode(boolean isCoast) {
        var config = new MotorOutputConfigs();
        config.NeutralMode = isCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        pullerTalon.getConfigurator().apply(config);
        pullerTalon.setControl(new NeutralOut());
    }

    @Override
    public void setHomed(boolean homed) {
        this.homed = homed;
    }
}
