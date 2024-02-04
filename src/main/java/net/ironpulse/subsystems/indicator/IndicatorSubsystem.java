package net.ironpulse.subsystems.indicator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import net.ironpulse.subsystems.indicator.IndicatorIO.Patterns;
import org.littletonrobotics.junction.Logger;

public class IndicatorSubsystem extends SubsystemBase {
    private final IndicatorIO io;
    private final IndicatorIOInputsAutoLogged inputs = new IndicatorIOInputsAutoLogged();
    private Patterns currentPattern = Patterns.NORMAL;
    @Getter
    private Patterns lastPattern = Patterns.NORMAL;

    private final Timer timer = new Timer();

    public IndicatorSubsystem(IndicatorIO io) {
        this.io = io;
    }

    public void setPattern(Patterns pattern) {
        lastPattern = currentPattern;
        currentPattern = pattern;
        io.setPattern(pattern);
        switch (pattern) {
            case FINISH_INTAKE, FINISH_SHOOT -> timer.restart();
            default -> {
            }
        }
    }

    @Override
    public void periodic() {
        switch (currentPattern) {
            case FINISH_INTAKE, FINISH_SHOOT -> resetLed();
            default -> {
            }
        }
        io.updateInputs(inputs);
        Logger.processInputs("Indicator", inputs);
    }

    private void resetLed() {
        if (!timer.hasElapsed(2)) return;
        setPattern(Patterns.NORMAL);
    }

    public void resetToLastPattern() {
        setPattern(lastPattern);
    }
}
