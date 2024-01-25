package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.ShooterSubsystem;

import static net.ironpulse.state.StateMachine.*;

public class PreShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final RobotContainer robotContainer;

    public PreShootCommand(RobotContainer robotContainer, ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.robotContainer = robotContainer;
    }

    @Override
    public void execute() {
        shooterSubsystem.getShootMotorLeft()
                .setVoltage(Constants.ShooterConstants.shootVoltage.magnitude());
        shooterSubsystem.getShootMotorRight()
                .setVoltage(Constants.ShooterConstants.shootVoltage.magnitude());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getShootMotorLeft().setVoltage(0);
        shooterSubsystem.getShootMotorRight().setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return robotContainer.getGlobalStateMachine().getCurrentState() == States.IDLE;
    }
}
