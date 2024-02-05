package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static net.ironpulse.Constants.ShooterConstants.ampDeployAngle;

public class AmpAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public AmpAimingCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.getIo()
                .setArmPosition(Radians.of(ampDeployAngle.in(Radians)));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmPosition(Rotations.of(0));
    }
}
