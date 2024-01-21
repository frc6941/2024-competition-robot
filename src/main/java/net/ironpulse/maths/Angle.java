package net.ironpulse.maths;

public class Angle {
    public static double continuousToPositive360(double angle) {
        var _angle = angle % 360;
        if (_angle < 0) _angle += 360;
        return _angle;
    }
}
