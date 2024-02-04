package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Rotations;
import static net.ironpulse.Constants.ShooterConstants.parallelDeployAngle;

public class ParallelAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ParallelAimingCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.getIo().setArmPosition(Rotations.of(parallelDeployAngle.in(Rotations)));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmPosition(Rotations.of(0));
    }
}
