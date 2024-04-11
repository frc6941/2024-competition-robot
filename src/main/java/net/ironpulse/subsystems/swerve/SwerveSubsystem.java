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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import net.ironpulse.Constants;
import net.ironpulse.drivers.LimelightHelpers;
import net.ironpulse.utils.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.wpilibj.RobotState.isAutonomous;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Microsecond;
import static edu.wpi.first.units.Units.Seconds;

public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final Measure<Time> simLoopPeriod = Microsecond.of(5);
    private Measure<Time> lastSimTime = Seconds.zero();
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        CommandScheduler.getInstance().registerSubsystem(this);
        configurePathPlanner();
        var response = this.getPigeon2().clearStickyFaults();
        if (response.isError())
            System.out.println("Pigeon2 failed sticky fault clearing with error" + response);
        for (int i = 0; i < 4; i++) {
            response = this.getModule(i).getCANcoder().clearStickyFaults();
            if (response.isError())
                System.out.println("Swerve CANCoder " + i + " failed sticky fault clearing with error" + response);
            response = this.getModule(i).getDriveMotor().clearStickyFaults();
            if (response.isError())
                System.out.println("Swerve Drive " + i + " failed sticky fault clearing with error" + response);
            response = this.getModule(i).getSteerMotor().clearStickyFaults();
            if (response.isError())
                System.out.println("Swerve Steer " + i + " failed sticky fault clearing with error" + response);
        }
        m_pigeon2.reset();
        if (!Utils.isSimulation()) return;
        startSimThread();
    }

    @Override
    public void periodic() {
        // Do not do: good enough
        if (isAutonomous()) {
            return;
        }
        boolean rejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", m_pigeon2.getAngle(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (Math.abs(m_pigeon2.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            rejectUpdate = true;
        }
        if (mt2.tagCount == 0) {
            rejectUpdate = true;
        }
        if (!rejectUpdate) {
            setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 9999999));
            addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
        }
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
