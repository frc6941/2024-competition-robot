package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class ShooterDownCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ShooterDownCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.getIo()
                .setShooterVoltage(Constants.ShooterConstants.manualAimingVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setShooterVoltage(Volts.zero());
    }
}
