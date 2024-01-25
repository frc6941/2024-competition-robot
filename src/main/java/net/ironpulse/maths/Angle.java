package net.ironpulse.maths;

public class Angle {
    public static double continuousToPositive360(double angle) {
        return (angle+360)%360;
    }
}
