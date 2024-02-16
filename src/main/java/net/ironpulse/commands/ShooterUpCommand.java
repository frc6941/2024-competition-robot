package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class ShooterUpCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ShooterUpCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.getIo()
                .setShooterVoltage(Constants.ShooterConstants.manualAimingVoltage.mutableCopy().negate());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setShooterVoltage(Volts.zero());
    }
}
