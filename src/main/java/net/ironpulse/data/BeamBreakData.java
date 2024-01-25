package net.ironpulse.data;

public record BeamBreakData(
        boolean intakerBeamBreak,
        boolean indexerBeamBreak,
        boolean shooterLeftBeamBreak,
        boolean shooterRightBeamBreak
) {
}
