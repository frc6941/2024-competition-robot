package net.ironpulse.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class ClimbManualShooterUpCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ClimbManualShooterUpCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        shooterSubsystem.getIo()
                .setPullerBrakeMode(true);
    }

    @Override
    public void execute() {
        var voltage = Constants.ShooterConstants.shooterUpDownVoltage.mutableCopy().negate();
        shooterSubsystem.getIo()
                .setPullerVoltage(Volts.zero());
        shooterSubsystem.getIo()
                .setArmVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setPullerBrakeMode(false);
        shooterSubsystem.getIo().setArmVoltage(Volts.zero());
    }
}
