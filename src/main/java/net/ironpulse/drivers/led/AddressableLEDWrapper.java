package net.ironpulse.drivers.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import lombok.Getter;
import lombok.Setter;
import net.ironpulse.drivers.led.patterns.SolidColorPattern;

@SuppressWarnings("PMD")
public class AddressableLEDWrapper {
    private final AddressableLED addressableLED;
    private final AddressableLEDBuffer buffer;

    @Setter
    @Getter
    private AddressableLEDPattern pattern = new SolidColorPattern(Color.kBlack);
    private final Notifier looper = new Notifier(this::update);

    @Setter
    private double intensity = 0.1;

    private double period = 0.05;

    public AddressableLEDWrapper(int port, int length) {
        addressableLED = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        addressableLED.setLength(length);
        addressableLED.setData(buffer);
        addressableLED.start();
        start(period);
    }

    public void setPeriod(double period) {
        this.period = period;
        stop();
        start(period);
    }

    public void start(double period) {
        looper.startPeriodic(period);
    }

    public void update() {
        pattern.setLEDs(buffer);
        for (int i = 0; i < buffer.getLength(); i++) {
            var originalColor = buffer.getLED(i);
            var intensityAdjustedColor = new Color(
                    originalColor.red * intensity,
                    originalColor.green * intensity,
                    originalColor.blue * intensity
            );
            buffer.setLED(i, intensityAdjustedColor);
        }
        addressableLED.setData(buffer);
    }

    public void stop() {
        looper.stop();
    }
}
