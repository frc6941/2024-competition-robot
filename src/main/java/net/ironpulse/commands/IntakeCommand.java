package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.IntakerSubsystem;

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
        intakerSubsystem.getIntakerMotor()
                .setVoltage(Constants.IntakerConstants.intakeVoltage.magnitude());
        robotContainer.getGlobalState().transfer(RobotContainer.Actions.INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            robotContainer.getGlobalState().transfer(RobotContainer.Actions.INTERRUPT_INTAKE);
        intakerSubsystem.getIntakerMotor()
                .setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return robotContainer.getGlobalState().getCurrentState() == RobotContainer.States.PENDING;
    }
}
