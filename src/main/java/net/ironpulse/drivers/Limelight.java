package net.ironpulse.drivers;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class Limelight {
    private static final NetworkTable limelightTable = NetworkTableInstance
            .getDefault()
            .getTable("limelight");

    private static final NetworkTableEntry tv = limelightTable.getEntry("tv");
    private static final NetworkTableEntry tx = limelightTable.getEntry("tx");
    private static final NetworkTableEntry ty = limelightTable.getEntry("ty");
    private static final NetworkTableEntry tid = limelightTable.getEntry("tid");

    private static final NetworkTableEntry botPoseWPIBlue = limelightTable.getEntry("botpose_wpiblue");
    private static final NetworkTableEntry targetPoseCameraSpace = limelightTable.getEntry("targetpose_cameraspace");

    /**
     * @return whether there is a target on the camera.
     */
    public static boolean hasTarget() {
        return tv.getDouble(0) == 1;
    }

    /**
     * Get target if present
     *
     * @return Target object
     */
    public static Optional<AprilTagTarget> getTarget() {
        if (!hasTarget()) return Optional.empty();
        var rawRobotPose = botPoseWPIBlue.getDoubleArray(new double[7]);
        var rawTargetPose = targetPoseCameraSpace.getDoubleArray(new double[6]);
        var tagId = tid.getDouble(-1);
        return Optional.of(
                new AprilTagTarget(
                        tagId,
                        new Translation2d(tx.getDouble(0), ty.getDouble(0)),
                        Microseconds.of(rawRobotPose[6]),
                        new Pose3d(
                                new Translation3d(rawRobotPose[0], rawRobotPose[1], rawRobotPose[2]),
                                new Rotation3d(
                                        Radians.convertFrom(rawRobotPose[3], Degrees),
                                        Radians.convertFrom(rawRobotPose[4], Degrees),
                                        Radians.convertFrom(rawRobotPose[5], Degrees)
                                )
                        ),
                        new Pose3d(
                                new Translation3d(rawTargetPose[0], rawTargetPose[1], rawTargetPose[2]),
                                new Rotation3d(
                                        Radians.convertFrom(rawTargetPose[3], Degrees),
                                        Radians.convertFrom(rawTargetPose[4], Degrees),
                                        Radians.convertFrom(rawTargetPose[5], Degrees)
                                )
                        )
                )
        );
    }

    public record AprilTagTarget(
            double tagId,
            Translation2d position,
            Measure<Time> latency,
            Pose3d botPoseWPIBlue,
            Pose3d targetPoseCameraSpace
    ) {

    }
}