package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static net.ironpulse.Constants.ShooterConstants.shootVoltage;

public class PreShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final RobotContainer robotContainer;


    public PreShootCommand(ShooterSubsystem shooterSubsystem,
                           RobotContainer robotContainer) {
        this.shooterSubsystem = shooterSubsystem;
        this.robotContainer = robotContainer;
    }

    @Override
    public void initialize() {
        robotContainer.getIndicatorSubsystem()
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
        robotContainer.getIndicatorSubsystem().resetToLastPattern();
    }
}
