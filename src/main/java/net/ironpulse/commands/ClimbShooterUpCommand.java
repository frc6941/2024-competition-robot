package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

public class ClimbShooterUpCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ClimbShooterUpCommand(ShooterSubsystem shooterSubsystem) {
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
        if (shooterSubsystem.getInputs().armPosition.minus(Radians.of(2.48)).gt(Radians.of(0.04))) {
            voltage = Volts.zero();
        }
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
