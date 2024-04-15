package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class TestParallelAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final LoggedDashboardNumber deployAngle;

    public TestParallelAimingCommand(ShooterSubsystem shooterSubsystem, LoggedDashboardNumber angle) {
        this.shooterSubsystem = shooterSubsystem;
        deployAngle = angle;
    }

    @Override
    public void execute() {
        shooterSubsystem.getIo().setArmPosition(Radians.of(Degrees.of(deployAngle.get()).in(Radians)));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmPosition(Radians.zero());
    }
}