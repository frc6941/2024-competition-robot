package net.ironpulse.drivers.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface AddressableLEDPattern {
    void setLEDs(AddressableLEDBuffer buffer);

    default boolean isAnimated() {
        return false;
    }
}
