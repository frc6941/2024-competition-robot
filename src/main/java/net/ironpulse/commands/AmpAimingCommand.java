package net.ironpulse.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.ShooterSubsystem;

import static net.ironpulse.state.StateMachine.*;

public class AmpAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final RobotContainer robotContainer;

    public AmpAimingCommand(ShooterSubsystem shooterSubsystem, RobotContainer robotContainer) {
        this.shooterSubsystem = shooterSubsystem;
        this.robotContainer = robotContainer;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.getDeployMotor().setControl(
                new MotionMagicVoltage(Constants.ShooterConstants.ampDeployAngle.magnitude()));
        robotContainer.getGlobalState().transfer(Actions.SHOOT);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) return;
        robotContainer.getGlobalState().transfer(Actions.INTERRUPT_SHOOT);
    }
}
