package net.ironpulse.commands.autos;

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
    public void execute() {
        timer.start();
        shooterSubsystem.getShootMotorLeft()
                .setVoltage(Constants.ShooterConstants.shootVoltage.magnitude());
        shooterSubsystem.getShootMotorRight()
                .setVoltage(Constants.ShooterConstants.shootVoltage.magnitude());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getShootMotorLeft().setVoltage(0);
        shooterSubsystem.getShootMotorRight().setVoltage(0);
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }
}
