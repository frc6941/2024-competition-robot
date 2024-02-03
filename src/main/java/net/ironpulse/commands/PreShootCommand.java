package net.ironpulse.commands;

import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.IndicatorSubsystem;
import net.ironpulse.subsystems.ShooterSubsystem;

public class PreShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final RobotContainer robotContainer;

    public PreShootCommand(
            ShooterSubsystem shooterSubsystem,
            RobotContainer robotContainer) {
        this.shooterSubsystem = shooterSubsystem;
        this.robotContainer = robotContainer;
    }

    @Override
    public void initialize() {
        robotContainer.getIndicatorSubsystem()
                .setPattern(IndicatorSubsystem.Patterns.SHOOTING);
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
        shooterSubsystem.getShootMotorLeft()
                .setVoltage(Constants.ShooterConstants.shooterConstantVoltage.magnitude());
        if (!interrupted) return;
        robotContainer.getIndicatorSubsystem().resetToLastPattern();
    }
}
