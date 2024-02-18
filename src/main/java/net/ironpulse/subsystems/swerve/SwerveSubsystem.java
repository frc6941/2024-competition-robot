package net.ironpulse.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.utils.LocalADStarAK;
import net.ironpulse.utils.Utils;
import net.ironpulse.utils.swerve.BetterSwerveKinematics;
import net.ironpulse.utils.swerve.BetterSwerveModuleState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.RobotState.isAutonomous;
import static net.ironpulse.Constants.SwerveConstants.*;

public class SwerveSubsystem extends SubsystemBase {
    public static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;

    private final BetterSwerveKinematics kinematics = new BetterSwerveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions =
            new SwerveModulePosition[]{
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    private Measure<Angle> lastHeadingRadians;
    private final PIDController thetaController;

    public SwerveSubsystem(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        // Configure heading PID TODO
        thetaController = new PIDController(0.4, 0, 0.01);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Start threads (no-op for each if no signals have been created)
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                () -> kinematics.toChassisSpeeds(getModuleStates()),
                this::runVelocity,
                new HolonomicPathFollowerConfig(
                        maxLinearSpeed.magnitude(),
                        driveBaseRadius.magnitude(),
                        new ReplanningConfig()
                ),
                Utils::flip,
                this
        );
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                activePath -> Logger.recordOutput(
                        "Odometry/Trajectory", activePath.toArray(new Pose2d[0])));
        PathPlannerLogging.setLogTargetPoseCallback(
                targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

        // Configure SysId
        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                state -> Logger.recordOutput("Drive/SysIdState", state.toString())
                        ),
                        new SysIdRoutine.Mechanism(
                                voltage -> Arrays
                                        .stream(modules)
                                        .forEach(module -> module.runCharacterization(voltage.in(Volts))),
                                null,
                                this)
                );

        // Zero pigeon
        zeroGyro();
    }

    public void zeroGyro() {
        this.gyroIO.zeroGyro();
        setPose(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        for (var module : modules) {
            module.updateInputs();
        }

        odometryLock.unlock();
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }

        // Update pose
        Limelight.getTarget()
                .ifPresent(target -> {
                    if (isAutonomous()) {
                        return;
                    }
                    var distanceToTag = target.targetPoseCameraSpace().relativeTo(target.botPoseWPIBlue()).getTranslation().getNorm();
                    var distanceToTag2 = distanceToTag * distanceToTag;
                    var defaultDevs = VecBuilder.fill(0.3, 0.3, Math.toRadians(45));
                    Matrix<N3, N1> visionStdMatBuilder = new Matrix<>(Nat.N3(), Nat.N1());
                    visionStdMatBuilder.set(0, 0, defaultDevs.get(0, 0) * distanceToTag2);
                    visionStdMatBuilder.set(1, 0, defaultDevs.get(1, 0) * distanceToTag2);
                    visionStdMatBuilder.set(2, 0, Math.atan(Math.tan(defaultDevs.get(2, 0)) * distanceToTag2 * distanceToTag));
                    addVisionMeasurement(
                            target.botPoseWPIBlue().toPose2d(),
                            Microseconds.of(Logger.getTimestamp()).minus(target.latency()).in(Seconds),
                            visionStdMatBuilder
                    );
                });

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }
        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[]{});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]{});
        }

        // Update odometry
        double[] sampleTimestamps =
                modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                        new SwerveModulePosition(
                                modulePositions[moduleIndex].distanceMeters
                                        - lastModulePositions[moduleIndex].distanceMeters,
                                modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        System.out.println("Running velocity -> " + speeds.omegaRadiansPerSecond);
        // Heading correction (see https://github.com/BroncBotz3481/YAGSL/blob/main/swervelib/SwerveDrive.java#L475)
        // TODO: swerveDrivePoseEstimator.getEstimatedPosition().getRotation() instead of rawGyroRotation?
        if (Math.abs(speeds.omegaRadiansPerSecond) < HEADING_CORRECTION_DEADBAND
                && (Math.abs(speeds.vxMetersPerSecond) > HEADING_CORRECTION_DEADBAND
                || Math.abs(speeds.vyMetersPerSecond) > HEADING_CORRECTION_DEADBAND)) {
            speeds.omegaRadiansPerSecond =
                    thetaController.calculate(poseEstimator.getEstimatedPosition().getRotation().getRadians(), lastHeadingRadians.magnitude())
                            * getMaxAngularSpeedRadPerSec();
            System.out.println(
                    "rawGyro -> " + rawGyroRotation.getRadians() + " lastHeading -> " + lastHeadingRadians.magnitude() +
                            " postEstimator -> " + poseEstimator.getEstimatedPosition().getRotation().getRadians() +
                            " correctedSpeed -> " + thetaController.calculate(poseEstimator.getEstimatedPosition().getRotation().getRadians(), lastHeadingRadians.magnitude())
                            * getMaxAngularSpeedRadPerSec()
            );
        } else {
            lastHeadingRadians = Radians.of(rawGyroRotation.getRadians());
        }

        // Logging purposes
        SwerveModuleState[] rawStates = new SwerveModuleState[4];
        SwerveModuleState[] rawOptimizedStates = new SwerveModuleState[4];

        // Calculate module setpoints
        BetterSwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        BetterSwerveKinematics.desaturateWheelSpeeds(setpointStates, maxLinearSpeed.magnitude());

        // Send setpoints to modules
        BetterSwerveModuleState[] optimizedSetpointStates = new BetterSwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i], true);
            rawStates[i] = new SwerveModuleState(setpointStates[i].speedMetersPerSecond, setpointStates[i].angle);
            rawOptimizedStates[i] = new SwerveModuleState(optimizedSetpointStates[i].speedMetersPerSecond, optimizedSetpointStates[i].angle);
        }

        // Log setpoint states
        Logger.recordOutput("SwerveStates/Setpoints", rawStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", rawOptimizedStates);
    }

    /**
     * Stops the drive.
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /**
     * Returns a command to run a dynamic test in the specified direction.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current odometry rotation.
     */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /**
     * Resets the current odometry pose.
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp  The timestamp of the vision measurement in seconds.
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
    }

    /**
     * Returns the maximum linear speed in meters per sec.
     */
    public double getMaxLinearSpeedMetersPerSec() {
        return maxLinearSpeed.magnitude();
    }

    /**
     * Returns the maximum angular speed in radians per sec.
     */
    public double getMaxAngularSpeedRadPerSec() {
        return maxAngularSpeed.magnitude();
    }

    /**
     * Returns an array of module translations.
     */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[]{
                new Translation2d(trackWidthX.magnitude() / 2.0, trackWidthY.magnitude() / 2.0),
                new Translation2d(trackWidthX.magnitude() / 2.0, -trackWidthY.magnitude() / 2.0),
                new Translation2d(-trackWidthX.magnitude() / 2.0, trackWidthY.magnitude() / 2.0),
                new Translation2d(-trackWidthX.magnitude() / 2.0, -trackWidthY.magnitude() / 2.0)
        };
    }
}