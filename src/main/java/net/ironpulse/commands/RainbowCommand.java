package net.ironpulse.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;

public class RainbowCommand extends Command {
    private final IndicatorSubsystem indicatorSubsystem;

    private final Timer timer = new Timer();

    public RainbowCommand(IndicatorSubsystem indicatorSubsystem) {
        this.indicatorSubsystem = indicatorSubsystem;
    }

    @Override
    public void initialize() {
        timer.restart();
        indicatorSubsystem.setPattern(IndicatorIO.Patterns.SHOULD_AMPLIFY);
    }

    @Override
    public void end(boolean interrupted) {
        indicatorSubsystem.resetToLastPattern();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }
}
