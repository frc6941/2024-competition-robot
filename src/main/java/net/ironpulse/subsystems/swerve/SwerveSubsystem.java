package net.ironpulse.subsystems.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import net.ironpulse.Constants;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.utils.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.RobotState.isAutonomous;

public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final Measure<Time> simLoopPeriod = Microsecond.of(5);
    private Measure<Time> lastSimTime = Seconds.of(0);
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        CommandScheduler.getInstance().registerSubsystem(this);
        configurePathPlanner();
        if (!Utils.isSimulation()) return;
        startSimThread();
    }

    @Override
    public void periodic() {
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
    }

    private void configurePathPlanner() {
        var driveBaseRadius = 0.0;
        for (var moduleLocation : m_moduleLocations)
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose,
                this::seedFieldRelative,
                this::getCurrentRobotChassisSpeeds,
                speeds -> this.setControl(autoRequest.withSpeeds(speeds)),
                new HolonomicPathFollowerConfig(
                        Constants.SwerveConstants.speedAt12Volts.magnitude(),
                        driveBaseRadius,
                        new ReplanningConfig()),
                net.ironpulse.utils.Utils::flip,
                this
        );
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                activePath -> Logger.recordOutput(
                        "Odometry/Trajectory", activePath.toArray(new Pose2d[0])));
        PathPlannerLogging.setLogTargetPoseCallback(
                targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));
    }

    /**
     * Apply a {@link SwerveRequest} to the SwerveSubsystem
     *
     * @param requestSupplier A lambda expression returns {@link SwerveRequest}
     * @return A {@link Command}
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Get current chassis speeds of the robot
     *
     * @return A {@link ChassisSpeeds} object
     * @see ChassisSpeeds
     */
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    @SuppressWarnings({"PMD.CloseResource", "resource"})
    private void startSimThread() {
        lastSimTime = Seconds.of(Utils.getCurrentTimeSeconds());

        var simNotifier = new Notifier(() -> {
            var currentTime = Seconds.of(Utils.getCurrentTimeSeconds());
            var deltaTime = currentTime.minus(lastSimTime);
            lastSimTime = currentTime;

            updateSimState(deltaTime.magnitude(), RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(simLoopPeriod.in(Seconds));
    }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }
}
