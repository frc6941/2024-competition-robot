package net.ironpulse.drivers.led.patterns;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import net.ironpulse.drivers.led.AddressableLEDPattern;

public class ChaosPattern implements AddressableLEDPattern {
    private boolean firstTime = true;

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        if (firstTime) {
            for (var index = 0; index < buffer.getLength(); index++) {
                buffer.setLED(index,
                        new Color(Math.random(), Math.random(), Math.random()));
            }
            firstTime = false;
        }
        for (var index = 0; index < buffer.getLength(); index++) {
            buffer.setLED(index,randomColorShift(buffer.getLED(index)));
        }
    }

    private Color randomColorShift(Color color) {
        return new Color(
                randomShift(color.red),
                randomShift(color.green),
                randomShift(color.blue)
        );
    }

    private double randomShift(double value) {
        var sign = Math.random() >= 0.5 ? 1.0 : -1.0;
        var amount = Math.random() / 10;
        return MathUtil.clamp(value + sign * amount, 0, 1);
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
