package net.ironpulse.drivers.led.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import net.ironpulse.drivers.led.AddressableLEDPattern;

public class SolidColorPattern implements AddressableLEDPattern {
    private final Color color;

    public SolidColorPattern(Color color) {
        this.color = color;
    }

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        for (int index = 0; index < buffer.getLength(); index++){
            buffer.setLED(index, color);
        }
    }
}
