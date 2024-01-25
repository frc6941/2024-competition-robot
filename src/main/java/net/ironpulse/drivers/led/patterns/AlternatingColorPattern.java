package net.ironpulse.drivers.led.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import net.ironpulse.drivers.led.AddressableLEDPattern;

public class AlternatingColorPattern implements AddressableLEDPattern {
    private final Color[] colors;

    public AlternatingColorPattern(Color[] colors) {
        this.colors = colors;
    }

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        for (var index = 0; index < buffer.getLength(); index++) {
            buffer.setLED(index, colors[index % colors.length]);
        }
    }
}
