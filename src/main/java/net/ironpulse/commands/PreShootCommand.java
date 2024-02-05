package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static net.ironpulse.Constants.ShooterConstants.shootVoltage;

public class PreShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;


    public PreShootCommand(ShooterSubsystem shooterSubsystem,
                           IndicatorSubsystem indicatorSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
    }

    @Override
    public void initialize() {
        indicatorSubsystem
                .setPattern(IndicatorIO.Patterns.SHOOTING);
    }

    @Override
    public void execute() {
        shooterSubsystem.getIo().setShooterVoltage(shootVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setShooterVoltage(Constants.ShooterConstants.shooterConstantVoltage);
        if (!interrupted) return;
        indicatorSubsystem.resetToLastPattern();
    }
}
