package net.ironpulse.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static net.ironpulse.Constants.ShooterConstants.defaultShootVoltage;
import static net.ironpulse.Constants.ShooterConstants.shooterConstantVoltage;

public class AutoPreShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Timer timer = new Timer();

    public AutoPreShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        shooterSubsystem.getIo().setShooterVoltage(defaultShootVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setShooterVoltage(shooterConstantVoltage);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(20); // FIXME
    }
}