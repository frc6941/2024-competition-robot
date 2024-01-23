package net.ironpulse.drivers.led.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import net.ironpulse.drivers.led.AddressableLEDPattern;

public class ChasePattern implements AddressableLEDPattern {
    private final Color[] colors;
    private final int segmentWidth;
    private int offset;

    public ChasePattern(Color[] colors, int segmentWidth){
        this.colors = colors;
        this.segmentWidth = segmentWidth;
    }

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        var numberOfColors = colors.length;
        int effectiveIndex;
        int colorIndex;
        int bufferLength = buffer.getLength();
        for (var index = 0; index < bufferLength; index++){
            effectiveIndex = (index + offset) % bufferLength;
            colorIndex =(index / segmentWidth)% numberOfColors;
            buffer.setLED(effectiveIndex, colors[colorIndex]);
        }

        offset = (offset + 1) % bufferLength;
    }

    @Override
    public boolean isAnimated(){
        return true;
    }
}
