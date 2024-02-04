package net.ironpulse.subsystems.indicator;

import net.ironpulse.drivers.led.AddressableLEDWrapper;
import net.ironpulse.drivers.led.patterns.SolidColorPattern;

import static net.ironpulse.Constants.IndicatorConstants.LED_BUFFER_LENGTH;
import static net.ironpulse.Constants.IndicatorConstants.LED_PORT;

public class IndicatorIOARGB implements IndicatorIO {
    private final AddressableLEDWrapper led;
    private Patterns currentPattern = Patterns.NORMAL;

    public IndicatorIOARGB() {
        led = new AddressableLEDWrapper(
                LED_PORT,
                LED_BUFFER_LENGTH
        );
        led.setIntensity(1.0);
        led.start(0.02);
    }

    @Override
    public void updateInputs(IndicatorIOInputs inputs) {
        inputs.currentPattern = currentPattern;
    }

    @Override
    public void setPattern(Patterns pattern) {
        currentPattern = pattern;
        if (pattern == Patterns.NORMAL) {
            led.setPattern(new SolidColorPattern(allianceColor()));
            return;
        }
        led.setPattern(pattern.pattern);
    }
}
