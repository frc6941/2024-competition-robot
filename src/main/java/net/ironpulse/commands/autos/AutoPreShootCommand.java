package net.ironpulse.commands.autos;

import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.ShooterSubsystem;

public class AutoPreShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Timer timer = new Timer();

    public AutoPreShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        timer.restart();
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

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }
}
