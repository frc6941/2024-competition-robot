package net.ironpulse.drivers;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightHelpers {
    private static final NetworkTable limelightTable = NetworkTableInstance
            .getDefault()
            .getTable("limelight");

    private static final NetworkTableEntry tv = limelightTable.getEntry("tv");
    private static final NetworkTableEntry tx = limelightTable.getEntry("tx");
    private static final NetworkTableEntry ty = limelightTable.getEntry("ty");

    private static final NetworkTableEntry tl = limelightTable.getEntry("tl");

    private static final NetworkTableEntry botPose = limelightTable.getEntry("botpose");

    /**
     * @return whether there is a target on the camera.
     */
    public static boolean hasTarget() {
        return tv.getDouble(0) == 1;
    }

    /**
     * Get offset from crosshair to target
     * @return Horizontal and vertical offset from crosshair to target
     */
    public static Translation2d getTargetPosition() {
        return new Translation2d(tx.getDouble(0), ty.getDouble(0));
    }

    public static double getLatency() {
        return tl.getDouble(0);
    }

    public static Pose3d getBotPose() {
        var rawPose = botPose.getDoubleArray(new double[6]);
        return new Pose3d(
                new Translation3d(rawPose[0], rawPose[1], rawPose[2]),
                new Rotation3d(rawPose[3], rawPose[4], rawPose[5])
        );
    }
}
