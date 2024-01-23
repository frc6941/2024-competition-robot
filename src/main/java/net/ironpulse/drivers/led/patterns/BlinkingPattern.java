package net.ironpulse.drivers.led.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import net.ironpulse.drivers.led.AddressableLEDPattern;

public class BlinkingPattern implements AddressableLEDPattern {
    private final AddressableLEDPattern onPattern;
    private final AddressableLEDPattern offPattern;
    private final double interval;
    private boolean on = true;
    private double lastChange;

    /**
     *
     * @param onColor color for when the blink is on.
     * @param interval time in seconds between changes.
     */
    public BlinkingPattern(Color onColor, double interval) {
        onPattern = new SolidColorPattern(onColor);
        offPattern = new SolidColorPattern(Color.kBlack);
        this.interval = interval;
    }

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        var timestamp = Timer.getFPGATimestamp();
        if (timestamp - lastChange > interval) {
            on = !on;
            lastChange = timestamp;
        }

        if (on) {
            onPattern.setLEDs(buffer);
            return;
        }
        offPattern.setLEDs(buffer);
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
