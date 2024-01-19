package net.ironpulse.data;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;

public record AprilTagTarget(
        Translation2d position,
        Measure<Time> latency,
        Pose3d botPose
) {
}
