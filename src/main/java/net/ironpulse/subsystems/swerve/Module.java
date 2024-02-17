package net.ironpulse.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import lombok.Getter;
import net.ironpulse.Constants;
import net.ironpulse.utils.swerve.BetterSwerveModuleState;
import net.ironpulse.utils.swerve.SwerveOptimizer;
import org.littletonrobotics.junction.Logger;

import static net.ironpulse.Constants.SwerveConstants.*;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int id;

    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController driveFeedback;
    private final PIDController turnFeedback;
    private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
    private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
    private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute
    /**
     * -- GETTER --
     * Returns the module positions received this cycle.
     */
    @Getter
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[]{};
    private BetterSwerveModuleState lastModuleState = new BetterSwerveModuleState();
    private boolean isOpenLoop = false;

    public Module(ModuleIO io, int id) {
        this.io = io;
        this.id = id;

        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                driveFeedforward = new SimpleMotorFeedforward(0.125719, 0.1259525);
                driveFeedback = new PIDController(0.1, 0.0, 0.0);
                turnFeedback = new PIDController(2, 0.0, 0.0);
                break;
            case SIM:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
                driveFeedback = new PIDController(0.1, 0.0, 0.0);
                turnFeedback = new PIDController(10.0, 0.0, 0.0);
                break;
            default:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
                driveFeedback = new PIDController(0.0, 0.0, 0.0);
                turnFeedback = new PIDController(0.0, 0.0, 0.0);
                break;
        }

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
        setBrakeMode(true);
    }

    /**
     * Update inputs without running the rest of the periodic logic. This is useful since these
     * updates need to be properly thread-locked.
     */
    public void updateInputs() {
        io.updateInputs(inputs);
    }

    public void periodic() {
        Logger.processInputs("Drive/Module" + id, inputs);

        // On first cycle, reset relative turn encoder
        // Wait until absolute angle is nonzero in case it wasn't initialized yet
        if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
            turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
        }

        if (!isOpenLoop) {
            // Run closed loop turn control
            if (angleSetpoint != null) {
                io.setTurnVoltage(
                        turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

                // Run closed loop drive control
                // Only allowed if closed loop turn control is running
                if (speedSetpoint != null) {
                    // Scale velocity based on turn error
                    //
                    // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
                    // towards the setpoint, its velocity should increase. This is achieved by
                    // taking the component of the velocity in the direction of the setpoint.
                    double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());

                    // Run drive controller
                    double velocityRadPerSec = adjustSpeedSetpoint / Constants.SwerveConstants.wheelRadius.magnitude();
                    io.setDriveVoltage(
                            driveFeedforward.calculate(velocityRadPerSec)
                                    + driveFeedback.calculate(inputs.driveVelocity.magnitude(), velocityRadPerSec));
                }
            }
        }

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (var i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * Constants.SwerveConstants.wheelRadius.magnitude();
            Rotation2d angle =
                    inputs.odometryTurnPositions[i].plus(
                            turnRelativeOffset != null ? turnRelativeOffset : new Rotation2d());
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    /**
     * Runs the module with the specified setpoint state. Returns the optimized state.
     */
    public BetterSwerveModuleState runSetpoint(BetterSwerveModuleState state, boolean isOpenLoop) {
        this.isOpenLoop = isOpenLoop;
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null
        var optimizedState = SwerveOptimizer.optimize(state, getAngle(), Units.radiansToDegrees(lastModuleState.omegaRadPerSecond * (isOpenLoop ? MODULE_STEER_FF_OL : MODULE_STEER_FF_CL) * 0.065));

        // Anti-jitter
        SwerveOptimizer.antiJitter(optimizedState, lastModuleState);

        setModuleState(optimizedState, isOpenLoop);

        lastModuleState = optimizedState;

        return optimizedState;
    }

    private void setModuleState(BetterSwerveModuleState state, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percent = state.speedMetersPerSecond / maxLinearSpeed.magnitude();
            io.setDriveVoltage(percent * 12);
        } else {
            System.out.println("Do not use close loop driving. It is prone to cause problems.");
            return;
            // Update setpoints, controllers run in "periodic"
//            angleSetpoint = optimizedState.angle;
//            speedSetpoint = state.speedMetersPerSecond;
//            driveFeedback.setSetpoint(state.speedMetersPerSecond);
//            io.setDriveVoltage(driveFeedback.calculate(inputs.driveVelocity.magnitude() * Math.PI * 2 * wheelRadius.magnitude() / DRIVE_GEAR_RATIO));
//            io.setDriveVoltage(driveFeedback.calculate(inputs.driveVelocity.magnitude() / wheelRadius.magnitude()));
//            System.out.println(inputs.driveVelocity.magnitude() * Math.PI * 2 * wheelRadius.magnitude() / DRIVE_GEAR_RATIO + " vs " + inputs.driveVelocity.magnitude() / wheelRadius.magnitude());
            // or could be driveVelocity.magnitude() / wheelRadius.magnitude()
        }
        turnFeedback.setSetpoint(state.angle.getRadians());
        io.setTurnVoltage(turnFeedback.calculate(getAngle().getRadians()) + MODULE_STEER_FF_OL * state.omegaRadPerSecond);
    }

    /**
     * Runs the module with the specified voltage while controlling to zero degrees.
     */
    public void runCharacterization(double volts) {
        // Closed loop turn control
        angleSetpoint = new Rotation2d();

        // Open loop drive control
        io.setDriveVoltage(volts);
        speedSetpoint = null;
    }

    /**
     * Disables all outputs to motors.
     */
    public void stop() {
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);

        // Disable closed loop control for turn and drive
        angleSetpoint = null;
        speedSetpoint = null;
    }

    /**
     * Sets whether brake mode is enabled.
     */
    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    /**
     * Returns the current turn angle of the module.
     */
    public Rotation2d getAngle() {
        if (turnRelativeOffset == null) {
            return new Rotation2d();
        } else {
            return inputs.turnPosition.plus(turnRelativeOffset);
        }
    }

    /**
     * Returns the current drive position of the module in meters.
     */
    public double getPositionMeters() {
        return inputs.drivePosition.times(Constants.SwerveConstants.wheelRadius).magnitude();
    }

    /**
     * Returns the current drive velocity of the module in meters per second.
     */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocity.times(Constants.SwerveConstants.wheelRadius).magnitude();
    }

    /**
     * Returns the module position (turn angle and drive position).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /**
     * Returns the module state (turn angle and drive velocity).
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /**
     * Returns the timestamps of the samples received this cycle.
     */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /**
     * Returns the drive velocity in radians/sec.
     */
    public double getCharacterizationVelocity() {
        return inputs.driveVelocity.magnitude();
    }
}