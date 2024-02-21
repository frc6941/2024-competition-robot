package net.ironpulse.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class ShooterIOSim implements ShooterIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    private final DCMotorSim leftShooterTalonSim = new DCMotorSim(
            DCMotor.getFalcon500(1),
            6.75, 0.025
    );
    private final DCMotorSim rightShooterTalonSim = new DCMotorSim(
            DCMotor.getFalcon500(1),
            6.75, 0.025
    );
    private final DCMotorSim armTalonSim = new DCMotorSim(
            DCMotor.getFalcon500(1),
            6.75, 0.025
    );
    private final DCMotorSim pullerTalonSim = new DCMotorSim(
            DCMotor.getFalcon500(1),
            6.75, 0.025
    );

    private boolean homed = false;

    private Measure<Voltage> leftShooterAppliedVoltage = Volts.zero();
    private Measure<Voltage> rightShooterAppliedVoltage = Volts.zero();
    private Measure<Voltage> armAppliedVoltage = Volts.zero();
    private Measure<Voltage> pullerAppliedVoltage = Volts.zero();

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        leftShooterTalonSim.update(LOOP_PERIOD_SECS);
        rightShooterTalonSim.update(LOOP_PERIOD_SECS);
        armTalonSim.update(LOOP_PERIOD_SECS);
        pullerTalonSim.update(LOOP_PERIOD_SECS);

        inputs.leftShooterVelocity =
                RadiansPerSecond.of(leftShooterTalonSim.getAngularVelocityRadPerSec());
        inputs.leftShooterPosition =
                Radians.of(leftShooterTalonSim.getAngularPositionRad());
        inputs.leftShooterAppliedVoltage =
                leftShooterAppliedVoltage;
        inputs.leftShooterSupplyCurrent =
                Amps.of(leftShooterTalonSim.getCurrentDrawAmps());

        inputs.rightShooterVelocity =
                RadiansPerSecond.of(rightShooterTalonSim.getAngularVelocityRadPerSec());
        inputs.rightShooterPosition =
                Radians.of(rightShooterTalonSim.getAngularPositionRad());
        inputs.rightShooterAppliedVoltage =
                rightShooterAppliedVoltage;
        inputs.rightShooterSupplyCurrent =
                Amps.of(rightShooterTalonSim.getCurrentDrawAmps());

        inputs.armPosition =
                Radians.of(armTalonSim.getAngularPositionRad());
        inputs.armAppliedVoltage =
                armAppliedVoltage;
        inputs.armSupplyCurrent =
                Amps.of(armTalonSim.getCurrentDrawAmps());

        inputs.pullerPosition =
                Radians.of(pullerTalonSim.getAngularPositionRad());
        inputs.pullerAppliedVoltage =
                pullerAppliedVoltage;
        inputs.pullerSupplyCurrent =
                Amps.of(pullerTalonSim.getCurrentDrawAmps());

        inputs.homed = homed;
    }

    @Override
    public void setShooterVoltage(Measure<Voltage> volts) {
        leftShooterAppliedVoltage = volts;
        rightShooterAppliedVoltage = leftShooterAppliedVoltage.mutableCopy().negate();

        leftShooterTalonSim.setInputVoltage(leftShooterAppliedVoltage.magnitude());
        rightShooterTalonSim.setInputVoltage(rightShooterAppliedVoltage.magnitude());
    }

    @Override
    public void setArmVoltage(Measure<Voltage> volts) {
        armAppliedVoltage = volts;
        armTalonSim.setInputVoltage(volts.magnitude());
    }

    @Override
    public void setPullerVoltage(Measure<Voltage> volts) {
        pullerAppliedVoltage = volts;
        pullerTalonSim.setInputVoltage(volts.magnitude());
    }

    @Override
    public void setArmHome(Measure<Angle> rad) {
        setArmPosition(rad);
    }

    @Override
    public void setArmPosition(Measure<Angle> rad) {
        armTalonSim.setState(rad.magnitude(), 0);
    }

    @Override
    public void setArmBrakeMode(boolean isCoast) {

    }

    @Override
    public void setPullerBrakeMode(boolean isCoast) {
        
    }

    @Override
    public void setHomed(boolean homed) {
        this.homed = homed;
    }
}
