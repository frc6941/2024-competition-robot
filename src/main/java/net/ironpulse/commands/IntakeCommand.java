package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.IntakerSubsystem;

import static net.ironpulse.state.StateMachine.Actions;
import static net.ironpulse.state.StateMachine.States;

public class IntakeCommand extends Command {
    private final IntakerSubsystem intakerSubsystem;
    private final RobotContainer robotContainer;

    public IntakeCommand(RobotContainer robotContainer, IntakerSubsystem intakerSubsystem) {
        this.intakerSubsystem = intakerSubsystem;
        this.robotContainer = robotContainer;
        addRequirements(intakerSubsystem);
    }

    @Override
    public void execute() {
        if (isFinished()) return;
        intakerSubsystem.getIntakerMotor()
                .setVoltage(Constants.IntakerConstants.intakeVoltage.magnitude());
        robotContainer.getGlobalStateMachine().transfer(Actions.INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            robotContainer.getGlobalStateMachine().transfer(Actions.INTERRUPT_INTAKE);
        intakerSubsystem.getIntakerMotor()
                .setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return robotContainer.getGlobalStateMachine().getCurrentState() == States.PENDING;
    }
}
