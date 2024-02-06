package net.ironpulse.commands;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Radians;

public class ParallelAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Measure<Angle> deployAngle;

    public ParallelAimingCommand(ShooterSubsystem shooterSubsystem, Measure<Angle> angle) {
        this.shooterSubsystem = shooterSubsystem;
        deployAngle = angle;
    }

    @Override
    public void execute() {
        shooterSubsystem.getIo().setArmPosition(Radians.of(deployAngle.in(Radians)));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmPosition(Radians.zero(), 1);
    }
}