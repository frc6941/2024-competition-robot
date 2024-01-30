package net.ironpulse.commands;

import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.ShooterSubsystem;

public class PreShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public PreShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.getShootMotorLeft()
                .setVoltage(Constants.ShooterConstants.shootVoltage.magnitude());
        shooterSubsystem.getShootMotorRight()
                .setControl(
                        new Follower(Constants.ShooterConstants.SHOOTER_L_MOTOR_ID,
                                true)
                );
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getShootMotorLeft().setVoltage(0);
    }
}
