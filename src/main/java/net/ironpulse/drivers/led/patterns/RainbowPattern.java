package net.ironpulse.drivers.led.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import net.ironpulse.drivers.led.AddressableLEDPattern;

public class RainbowPattern implements AddressableLEDPattern {
    private int firstHue = 0;

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        int currentHue;
        for (var index = 0; index < buffer.getLength(); index++){
            currentHue = (firstHue + (index * 180 / buffer.getLength())) % 180;
            buffer.setHSV(index, currentHue, 255, 128);
        }

        firstHue = (firstHue + 3) % 180;
    }

    @Override
    public boolean isAnimated(){
        return true;
    }
}
