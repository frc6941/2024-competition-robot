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
    public void initialize() {
        shooterSubsystem.getIo()
                .setPullerBrakeMode(true);
    }

    @Override
    public void execute() {
        shooterSubsystem.getIo()
                .setArmVoltage(Constants.ShooterConstants.shooterUpDownVoltage.mutableCopy().negate());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setPullerBrakeMode(false);
        shooterSubsystem.getIo().setArmVoltage(Volts.zero());
    }
}
