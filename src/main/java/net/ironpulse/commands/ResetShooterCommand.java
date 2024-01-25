package net.ironpulse.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.ShooterSubsystem;

public class ResetShooterCommand extends Command {
    public final ShooterSubsystem shooterSubsystem;

    public ResetShooterCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.getDeployMotor().setControl(new MotionMagicVoltage(0));
    }
}
