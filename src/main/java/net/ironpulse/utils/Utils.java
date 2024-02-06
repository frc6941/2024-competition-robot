package net.ironpulse.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class Utils {
    public static boolean flip() {
        return DriverStation
                .getAlliance()
                .filter(alliance -> alliance == DriverStation.Alliance.Red)
                .isPresent();
    }
}
