package net.ironpulse.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class Utils {
    public static boolean flip() {
        return DriverStation
                .getAlliance()
                .filter(alliance -> alliance == DriverStation.Alliance.Red)
                .isPresent();
    }

    public static int sign(double value) {
        if (value < 0) return -1;
        else if (value == 0) return 0;
        else return 1;
    }
}
