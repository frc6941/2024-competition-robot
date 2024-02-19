package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class ClimbCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ClimbCommand(
            ShooterSubsystem shooterSubsystem
    ) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.getIo().setArmBrakeMode(true);
        shooterSubsystem.getIo().setPullerVoltage(Constants.ShooterConstants.pullVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmBrakeMode(false);
        shooterSubsystem.getIo()
                .setPullerVoltage(Volts.zero());
    }
}
