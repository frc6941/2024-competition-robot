package net.ironpulse.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.drivers.led.AddressableLEDPattern;
import net.ironpulse.drivers.led.patterns.BlinkingPattern;
import net.ironpulse.subsystems.IndicatorSubsystem;

public class FinishIntakeLEDCommand extends Command {
    private final IndicatorSubsystem indicatorSubsystem;

    private final Timer timer = new Timer();

    private AddressableLEDPattern previousPattern;

    public FinishIntakeLEDCommand(IndicatorSubsystem indicatorSubsystem) {
        this.indicatorSubsystem = indicatorSubsystem;
        addRequirements(indicatorSubsystem);
    }

    @Override
    public void initialize() {
        previousPattern = indicatorSubsystem.getLed().getPattern();
        indicatorSubsystem.setInOtherPatterns(true);
        timer.start();
        indicatorSubsystem.getLed()
                .setPattern(new BlinkingPattern(Color.kGreen, 0.2));
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
        indicatorSubsystem.setInOtherPatterns(false);
        indicatorSubsystem.getLed().setPattern(previousPattern);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }
}
