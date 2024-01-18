package net.ironpulse.maths;

record Range(double max, double min) {
    public boolean inRange(double value) {
        return value <= max && value >= min;
    }
}