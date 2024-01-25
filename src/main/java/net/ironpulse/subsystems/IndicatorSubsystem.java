package net.ironpulse.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;
import net.ironpulse.RobotContainer;
import net.ironpulse.drivers.led.AddressableLEDPattern;
import net.ironpulse.drivers.led.AddressableLEDWrapper;
import net.ironpulse.drivers.led.patterns.ScannerPattern;
import net.ironpulse.drivers.led.patterns.SolidColorPattern;
import net.ironpulse.state.StateMachine;

import static net.ironpulse.Constants.IndicatorConstants.LED_BUFFER_LENGTH;
import static net.ironpulse.Constants.IndicatorConstants.LED_PORT;

public class IndicatorSubsystem implements Subsystem {
    @Getter
    private final AddressableLEDWrapper led =
            new AddressableLEDWrapper(LED_PORT, LED_BUFFER_LENGTH);

    private final RobotContainer robotContainer;

    /**
     * This provides possibilities to add temporary patterns.
     * e.g. Temporarily blinking LEDs when receiving a note for 3 seconds.
     */
    @Setter
    private boolean inOtherPatterns = false;

    private StateMachine.States currentState = StateMachine.States.IDLE;

    public IndicatorSubsystem(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        CommandScheduler.getInstance().registerSubsystem(this);
        led.setPeriod(1);
        led.start(0.02);
    }

    @Override
    public void periodic() {
        if (inOtherPatterns) return;
        if (robotContainer.getGlobalStateMachine().getCurrentState() == currentState)
            return;
        switch (robotContainer.getGlobalStateMachine().getCurrentState()) {
            case IDLE -> led.setPattern(Patterns.IDLE.pattern);
            case PENDING -> led.setPattern(Patterns.PENDING.pattern);
            case INTAKING -> led.setPattern(Patterns.INTAKING.pattern);
            case SHOOTING -> led.setPattern(Patterns.SHOOTING.pattern);
        }
        currentState = robotContainer.getGlobalStateMachine().getCurrentState();
    }

    /**
     * Map states to LED patterns
     * <p>
     * To find more patterns, see {@link net.ironpulse.drivers.led.patterns}
     */
    @AllArgsConstructor
    private enum Patterns {
        // TODO Make more patterns
        IDLE(new SolidColorPattern(Color.kDarkBlue)),
        PENDING(new SolidColorPattern(Color.kDarkOrange)),
        INTAKING(new SolidColorPattern(Color.kDarkViolet)),
        SHOOTING(new ScannerPattern(Color.kRed, 1));

        private final AddressableLEDPattern pattern;
    }
}
