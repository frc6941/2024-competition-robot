package net.ironpulse.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import net.ironpulse.Constants;
import net.ironpulse.drivers.Limelight;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Microsecond;
import static edu.wpi.first.units.Units.Seconds;

public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final Measure<Time> simLoopPeriod = Microsecond.of(0.005);
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
        updatePoseEstimatorFromLimelight();
    }

    /**
     * Update {@link PoseEstimator} from Limelight Vision
     */
    private void updatePoseEstimatorFromLimelight() {
        // TODO: Verify this
        Limelight
                .getTarget()
                .ifPresent(target ->
                        addVisionMeasurement(
                                target.botPose().toPose2d(),
                                Timer.getFPGATimestamp() -
                                        target.latency().in(Seconds)
                        )
                );
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
                        new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        Constants.SwerveConstants.speedAt12Volts.magnitude(),
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> false,
                this
        );
    }

    /**
     * Apply a {@link SwerveRequest} to the SwerveSubsystem
     * @param requestSupplier A lambda expression returns {@link SwerveRequest}
     * @return A {@link Command}
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Get an auto created by PathPlanner editor through its name
     * @param autoName Auto name
     * @return A {@link Command}
     */
    public Command getAuto(String autoName) {
        return new PathPlannerAuto(autoName);
    }

    /**
     * Get current chassis speeds of the robot
     * @return A {@link ChassisSpeeds} object
     * @see ChassisSpeeds
     */
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        lastSimTime = Seconds.of(Utils.getCurrentTimeSeconds());

        @SuppressWarnings("PMD.CloseResource")
        var simNotifier = new Notifier(() -> {
            var currentTime = Seconds.of(Utils.getCurrentTimeSeconds());
            var deltaTime = currentTime.minus(lastSimTime);
            lastSimTime = currentTime;

            updateSimState(deltaTime.magnitude(), RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(simLoopPeriod.magnitude());
    }
}
