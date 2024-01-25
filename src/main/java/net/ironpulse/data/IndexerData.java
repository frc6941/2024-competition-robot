package net.ironpulse.data;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

public record IndexerData(
        Measure<Angle> indexerMotorPosition
) {
}
