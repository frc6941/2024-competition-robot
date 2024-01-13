package net.ironpulse.telemetries;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.Seconds;

@SuppressWarnings("PMD.AvoidDuplicateLiterals")
public class SwerveTelemetry {
    private final Measure<Velocity<Distance>> maxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     *
     * @param maxSpeed Maximum speed in meters per second
     */
    public SwerveTelemetry(Measure<Velocity<Distance>> maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub =
            table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    private final NetworkTable driveStats = inst.getTable("Drive");
    private final DoublePublisher velocityX =
            driveStats.getDoubleTopic("Velocity X").publish();
    private final DoublePublisher velocityY =
            driveStats.getDoubleTopic("Velocity Y").publish();
    private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
    private final DoublePublisher odomFreq =
            driveStats.getDoubleTopic("Odometry Frequency").publish();

    private Pose2d lastPose = new Pose2d();
    private Measure<Time> lastTime = Seconds.of(Utils.getCurrentTimeSeconds());

    private final Mechanism2d[] moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1),
    };

    private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
        moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };

    private final MechanismLigament2d[] moduleDirections = new MechanismLigament2d[] {
        moduleMechanisms[0]
                .getRoot("RootDirection", 0.5, 0.5)
                .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[1]
                .getRoot("RootDirection", 0.5, 0.5)
                .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[2]
                .getRoot("RootDirection", 0.5, 0.5)
                .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[3]
                .getRoot("RootDirection", 0.5, 0.5)
                .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    public void telemeterize(SwerveDrivetrain.SwerveDriveState state) {
        var pose = state.Pose;
        fieldTypePub.set("Field2d");
        fieldPub.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});

        var currentTime = Seconds.of(Utils.getCurrentTimeSeconds());
        var diffTime = currentTime.minus(lastTime);
        lastTime = currentTime;
        var distanceDiff = pose.minus(lastPose).getTranslation();
        lastPose = pose;

        var velocities = distanceDiff.div(diffTime.magnitude());

        speed.set(velocities.getNorm());
        velocityX.set(velocities.getX());
        velocityY.set(velocities.getY());
        odomFreq.set(1.0 / state.OdometryPeriod);

        for (int i = 0; i < 4; ++i) {
            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * maxSpeed.magnitude()));

            SmartDashboard.putData("Module " + i, moduleMechanisms[i]);
        }

        SignalLogger.writeDoubleArray(
                "odometry",
                new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
        SignalLogger.writeDouble("odom period", state.OdometryPeriod, "seconds");
    }
}
