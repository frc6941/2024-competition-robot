package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static net.ironpulse.Constants.ShooterConstants.shootVoltage;

public class PreShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public PreShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.getIo().setShooterVoltage(shootVoltage);
    }
}
