package net.ironpulse.drivers;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import net.ironpulse.data.AprilTagTarget;

import java.util.Optional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Radians;

public class Limelight {
    private static final NetworkTable limelightTable = NetworkTableInstance
            .getDefault()
            .getTable("limelight");

    private static final NetworkTableEntry tv = limelightTable.getEntry("tv");
    private static final NetworkTableEntry tx = limelightTable.getEntry("tx");
    private static final NetworkTableEntry ty = limelightTable.getEntry("ty");

    private static final NetworkTableEntry botPose = limelightTable.getEntry("botpose_wpiblue");
    private static final NetworkTableEntry targetPoseCameraSpace = limelightTable.getEntry("targetpose_cameraspace");

    /**
     * @return whether there is a target on the camera.
     */
    public static boolean hasTarget() {
        return tv.getDouble(0) == 1;
    }

    /**
     * Get target if present
     * @return Target object
     */
    public static Optional<AprilTagTarget> getTarget() {
        if (!hasTarget()) return Optional.empty();
        var rawPose = botPose.getDoubleArray(new double[7]);
        var rawTargetPose = targetPoseCameraSpace.getDoubleArray(new double[6]);
        return Optional.of(
                new AprilTagTarget(
                    new Translation2d(tx.getDouble(0), ty.getDouble(0)),
                    Microseconds.of(rawPose[6]),
                    new Pose3d(
                            new Translation3d(rawPose[0], rawPose[1], rawPose[2]),
                            new Rotation3d(
                                    Radians.convertFrom(rawPose[3], Degrees),
                                    Radians.convertFrom(rawPose[4], Degrees),
                                    Radians.convertFrom(rawPose[5], Degrees)
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
}
