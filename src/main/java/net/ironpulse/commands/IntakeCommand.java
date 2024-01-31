package net.ironpulse.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.IndicatorSubsystem;
import net.ironpulse.subsystems.IntakerSubsystem;

public class IntakeCommand extends Command {
    private final IntakerSubsystem intakerSubsystem;
    private final RobotContainer robotContainer;

    public IntakeCommand(
            RobotContainer robotContainer,
            IntakerSubsystem intakerSubsystem
    ) {
        this.intakerSubsystem = intakerSubsystem;
        this.robotContainer = robotContainer;
        addRequirements(intakerSubsystem);
    }

    @Override
    public void execute() {
        if (isFinished()) return;
        intakerSubsystem.getIntakerMotor()
                .setVoltage(Constants.IntakerConstants.intakeVoltage.magnitude());
    }

    @Override
    public void end(boolean interrupted) {
        intakerSubsystem.getIntakerMotor()
                .setVoltage(0);
        robotContainer.getDriverController().getHID()
                .setRumble(GenericHID.RumbleType.kBothRumble, 1);
        robotContainer.getIndicatorSubsystem()
                .setPattern(IndicatorSubsystem.Patterns.FINISH_INTAKE);
    }

    @Override
    public boolean isFinished() {
        return robotContainer.getBeamBreakSubsystem().getIndexerBeamBreak().get() &&
                !robotContainer.getBeamBreakSubsystem().getIntakerBeamBreak().get();
    }
}
