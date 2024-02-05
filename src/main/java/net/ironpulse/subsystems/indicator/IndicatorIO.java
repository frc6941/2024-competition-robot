package net.ironpulse.subsystems.indicator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import net.ironpulse.drivers.led.AddressableLEDPattern;
import net.ironpulse.drivers.led.patterns.BlinkingPattern;
import net.ironpulse.drivers.led.patterns.ScannerPattern;
import org.littletonrobotics.junction.AutoLog;

public interface IndicatorIO {
    /**
     * All available patterns.
     */
    enum Patterns {
        NORMAL(null),
        FINISH_INTAKE(new BlinkingPattern(Color.kGreen, 0.5)),
        SHOOTING(new ScannerPattern(Color.kRed, 2)),
        FINISH_SHOOT(new BlinkingPattern(Color.kRed, 0.5));

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
    default void updateInputs(IndicatorIOInputs inputs) {
    }

    /**
     * Set current pattern.
     */
    default void setPattern(Patterns pattern) {
    }
}
