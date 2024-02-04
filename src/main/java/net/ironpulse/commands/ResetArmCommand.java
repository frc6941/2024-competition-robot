package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

public class ResetArmCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ResetArmCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        shooterSubsystem.getIo().setHomed(false);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.getInputs().homed;
    }
}
