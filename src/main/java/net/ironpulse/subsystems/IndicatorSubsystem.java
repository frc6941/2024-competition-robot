package net.ironpulse.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import net.ironpulse.RobotContainer;
import net.ironpulse.drivers.led.AddressableLEDPattern;
import net.ironpulse.drivers.led.AddressableLEDWrapper;
import net.ironpulse.drivers.led.patterns.BlinkingPattern;

import static net.ironpulse.Constants.IndicatorConstants.*;

public class IndicatorSubsystem implements Subsystem {
    private final AddressableLEDWrapper led =
            new AddressableLEDWrapper(LED_PORT, LED_BUFFER_LENGTH);

    private final RobotContainer robotContainer;

    public IndicatorSubsystem(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        CommandScheduler.getInstance().registerSubsystem(this);
        led.setPeriod(1);
        led.start(0.02);
    }

    @Override
    public void periodic() {
        switch (robotContainer.getGlobalStateMachine().getCurrentState()) {
            case IDLE -> led.setPattern(Patterns.IDLE.pattern);
            case PENDING -> led.setPattern(Patterns.PENDING.pattern);
            case INTAKING -> led.setPattern(Patterns.INTAKING.pattern);
            case SHOOTING -> led.setPattern(Patterns.SHOOTING.pattern);
        }
    }

    private enum Patterns {
        // TODO Make more patterns
        IDLE(new BlinkingPattern(Color.kRed, 0.2)),
        PENDING(new BlinkingPattern(Color.kRed, 0.2)),
        INTAKING(new BlinkingPattern(Color.kRed, 0.2)),
        SHOOTING(new BlinkingPattern(Color.kRed, 0.2));

        private final AddressableLEDPattern pattern;

        Patterns(AddressableLEDPattern pattern) {
            this.pattern = pattern;
        }
    }
}
