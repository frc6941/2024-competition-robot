package net.ironpulse.commands.manuals;

import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.ShooterSubsystem;

public class ManualShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ManualShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.getShootMotorLeft()
                .setVoltage(Constants.IntakerConstants.intakeVoltage.magnitude());
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
