package net.ironpulse.utils;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.units.Units.Volts;

public class Utils {
    public static boolean autoIntaking = false;

    public static boolean blind = false;

    public static boolean armReachedClimb = false;

    public static Measure<Voltage> autoShootVoltage = Volts.of(-9);

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
