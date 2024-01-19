package net.ironpulse.drivers;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import net.ironpulse.data.AprilTagTarget;

import java.util.Optional;

import static edu.wpi.first.units.Units.Seconds;

public class Limelight {
    private static final NetworkTable limelightTable = NetworkTableInstance
            .getDefault()
            .getTable("limelight");

    private static final NetworkTableEntry tv = limelightTable.getEntry("tv");
    private static final NetworkTableEntry tx = limelightTable.getEntry("tx");
    private static final NetworkTableEntry ty = limelightTable.getEntry("ty");

    private static final NetworkTableEntry tl = limelightTable.getEntry("tl");
    private static final NetworkTableEntry cl = limelightTable.getEntry("cl");

    private static final NetworkTableEntry botPose = limelightTable.getEntry("botpose");

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
        var rawPose = botPose.getDoubleArray(new double[6]);
        return Optional.of(
                new AprilTagTarget(
                    new Translation2d(tx.getDouble(0), ty.getDouble(0)),
                    Seconds.of(tl.getDouble(0) + cl.getDouble(0)),
                    new Pose3d(
                            new Translation3d(rawPose[0], rawPose[1], rawPose[2]),
                            new Rotation3d(rawPose[3], rawPose[4], rawPose[5])
                    )
                )
        );
    }
}
