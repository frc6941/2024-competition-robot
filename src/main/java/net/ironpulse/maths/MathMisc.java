package net.ironpulse.maths;

public class MathMisc {
    public static int sign(double value) {
        if (value < 0) return -1;
        else if (value == 0) return 0;
        else return 1;
    }
}
