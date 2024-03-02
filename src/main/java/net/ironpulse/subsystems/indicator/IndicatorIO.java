package net.ironpulse.subsystems.indicator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import net.ironpulse.drivers.led.AddressableLEDPattern;
import net.ironpulse.drivers.led.patterns.BlinkingPattern;
import net.ironpulse.drivers.led.patterns.RainbowPattern;
import net.ironpulse.drivers.led.patterns.ScannerPattern;
import net.ironpulse.drivers.led.patterns.SolidColorPattern;
import org.littletonrobotics.junction.AutoLog;

public interface IndicatorIO {
    /**
     * All available patterns.
     */
    enum Patterns {
        NORMAL(null),
        FINISH_INTAKE(new BlinkingPattern(Color.kGreen, 0.1)),
        SHOOTING(new ScannerPattern(Color.kRed, 2)), // unused
        FINISH_SHOOT(new BlinkingPattern(Color.kRed, 0.5)),
        SHOULD_AMPLIFY(new RainbowPattern()),
        AIMING(new BlinkingPattern(Color.kYellow, 0.5)),
        AIMED(new SolidColorPattern(Color.kYellow)),
        CAN_CLIMB(new BlinkingPattern(Color.kPurple, 0.2)),
        CLIMBING(new BlinkingPattern(Color.kViolet, 0.7));

        public final AddressableLEDPattern pattern;

        Patterns(AddressableLEDPattern color) {
            this.pattern = color;
        }
    }

    /**
     * Returns alliance color.
     *
     * @return Current alliance color
     */
    default Color allianceColor() {
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
            case Blue -> Color.kBlue;
            case Red -> Color.kRed;
        };
    }

    @AutoLog
    class IndicatorIOInputs {
        public Patterns currentPattern;
    }

    /**
     * Updates the set of loggable inputs.
     */
    void updateInputs(IndicatorIOInputs inputs);

    /**
     * Set current pattern.
     */
    void setPattern(Patterns pattern);

    /**
     * Stops and starts indicator.
     */
    void reset();
}
