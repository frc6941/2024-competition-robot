package net.ironpulse.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static net.ironpulse.Constants.Logger.debug;
import static net.ironpulse.Constants.ShooterConstants.defaultShootVoltage;
import static net.ironpulse.Constants.ShooterConstants.shooterConstantVoltage;

public class AutoShooterSpeedUpCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Timer timer = new Timer();

    public AutoShooterSpeedUpCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        debug("AutoShooterSpeedUpCommand", "start");
        timer.restart();
    }

    @Override
    public void execute() {
        shooterSubsystem.getIo().setShooterVoltage(defaultShootVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        debug("AutoShooterSpeedUpCommand", "end; elapsed=" + timer.get());
        shooterSubsystem.getIo().setShooterVoltage(shooterConstantVoltage);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(15);
    }
}