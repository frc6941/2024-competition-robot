package net.ironpulse.maths;

public record Compare(double value1, double value2) {
    public boolean epsilonEqual(double epsilon) {
        return Math.abs(value1 - value2) < epsilon;
    }
}