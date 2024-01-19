package net.ironpulse.drivers;

import edu.wpi.first.wpilibj.AnalogInput;

public class BeamBreak {
    private boolean lastStatus;

    private boolean tripped;

    private boolean cleared;

    private final AnalogInput beamBreak;

    public BeamBreak(int channel) {
        beamBreak = new AnalogInput(channel);
    }

    public void update() {
        boolean value = get();
        tripped = value && !lastStatus;
        cleared = !value && lastStatus;
        lastStatus = value;
    }

    public boolean get() {
        return beamBreak.getVoltage() > 2.0;
    }

    public boolean wasTripped() {
        return tripped;
    }

    public boolean wasCleared() {
        return cleared;
    }
}