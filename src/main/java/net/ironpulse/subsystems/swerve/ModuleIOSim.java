package net.ironpulse.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private final DCMotorSim driveSim = new DCMotorSim(DCMotor.getFalcon500Foc(1), 6.75, 0.025);
    private final DCMotorSim turnSim = new DCMotorSim(DCMotor.getFalcon500Foc(1), 150.0 / 7.0, 0.004);

    private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
    private Measure<Voltage> driveAppliedVoltage = Volts.of(0);
    private Measure<Voltage> turnAppliedVoltage = Volts.of(0);

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(LOOP_PERIOD_SECS);
        turnSim.update(LOOP_PERIOD_SECS);

        inputs.drivePosition = Radians.of(driveSim.getAngularPositionRad());
        inputs.driveVelocity = RadiansPerSecond.of(driveSim.getAngularVelocityRadPerSec());
        inputs.driveAppliedVoltage = driveAppliedVoltage;
        inputs.driveCurrentAmps = new double[]{Math.abs(driveSim.getCurrentDrawAmps())};

        inputs.turnAbsolutePosition =
                new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
        inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnVelocity = RadiansPerSecond.of(turnSim.getAngularVelocityRadPerSec());
        inputs.turnAppliedVoltage = turnAppliedVoltage;
        inputs.turnCurrentAmps = new double[]{Math.abs(turnSim.getCurrentDrawAmps())};

        inputs.odometryTimestamps = new double[]{Timer.getFPGATimestamp()};
        inputs.odometryDrivePositionsRad = new double[]{inputs.drivePosition.magnitude()};
        inputs.odometryTurnPositions = new Rotation2d[]{inputs.turnPosition};
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVoltage = Volts.of(MathUtil.clamp(volts, -12.0, 12.0));
        driveSim.setInputVoltage(driveAppliedVoltage.magnitude());
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVoltage = Volts.of(MathUtil.clamp(volts, -12.0, 12.0));
        turnSim.setInputVoltage(turnAppliedVoltage.magnitude());
    }
}