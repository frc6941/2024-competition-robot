package net.ironpulse.drivers.led.patterns;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import lombok.Setter;
import net.ironpulse.drivers.led.AddressableLEDPattern;

public class IntensityPattern implements AddressableLEDPattern {
    private final Color highColor;
    private final Color lowColor;

    @Setter
    private double intensity;

    /**
     *
     * @param highColor Brightest color
     * @param intensity 0..1 with 1 being the color and 0 being black
     */
    public IntensityPattern(Color highColor, double intensity){
        this(Color.kBlack,highColor,intensity);
    }

    public IntensityPattern(Color lowColor, Color highColor, double intensity){
        this.highColor = highColor;
        this.lowColor = lowColor;
        this.intensity = intensity;
    }

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        for (var index = 0; index < buffer.getLength(); index++){
            buffer.setLED(index, new Color(
                    MathUtil.interpolate(lowColor.red, highColor.red, intensity),
                    MathUtil.interpolate(lowColor.green, highColor.green, intensity),
                    MathUtil.interpolate(lowColor.blue, highColor.blue, intensity)
                    )
            );
        }
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
