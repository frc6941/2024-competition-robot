package net.ironpulse.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.Getter;
import net.ironpulse.drivers.led.AddressableLEDPattern;
import net.ironpulse.drivers.led.AddressableLEDWrapper;
import net.ironpulse.drivers.led.patterns.BlinkingPattern;
import net.ironpulse.drivers.led.patterns.ScannerPattern;
import net.ironpulse.drivers.led.patterns.SolidColorPattern;

import static net.ironpulse.Constants.IndicatorConstants.LED_BUFFER_LENGTH;
import static net.ironpulse.Constants.IndicatorConstants.LED_PORT;

public class IndicatorSubsystem implements Subsystem {
    private final AddressableLEDWrapper led;
    private Patterns currentPattern = Patterns.NORMAL;

    @Getter
    private Patterns lastPattern = Patterns.NORMAL;

    private final Timer timer = new Timer();

    public IndicatorSubsystem() {
        CommandScheduler.getInstance().registerSubsystem(this);
        led = new AddressableLEDWrapper(
                LED_PORT,
                LED_BUFFER_LENGTH
        );
        led.setIntensity(1.0);
        led.start(0.02);
    }

    private static Color allianceColor() {
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
            case Blue -> Color.kBlue;
            case Red -> Color.kRed;
        };
    }

    public void setPattern(Patterns pattern) {
        if (pattern == Patterns.NORMAL) {
            lastPattern = currentPattern;
            currentPattern = pattern;
            led.setPattern(new SolidColorPattern(allianceColor()));
            return;
        }
        lastPattern = currentPattern;
        currentPattern = pattern;
        led.setPattern(pattern.pattern);
        switch (pattern) {
            case FINISH_INTAKE, FINISH_SHOOT -> timer.start();
            default -> {}
        }
    }

    @Override
    public void periodic() {
        switch (currentPattern) {
            case FINISH_INTAKE, FINISH_SHOOT -> resetLed();
            default -> {}
        }
    }

    private void resetLed() {
        if (!timer.hasElapsed(2)) return;
        setPattern(Patterns.NORMAL);
        timer.stop();
        timer.reset();
    }

    public void resetToLastPattern() {
        setPattern(lastPattern);
    }

    public enum Patterns {
        NORMAL(null),
        FINISH_INTAKE(new BlinkingPattern(Color.kGreen, 0.5)),
        SHOOTING(new ScannerPattern(Color.kRed, 2)),
        FINISH_SHOOT(new BlinkingPattern(Color.kRed, 0.5));

        public final AddressableLEDPattern pattern;

        Patterns(AddressableLEDPattern color) {
            this.pattern = color;
        }
    }
}
