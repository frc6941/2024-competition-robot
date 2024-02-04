package net.ironpulse.commands.manuals;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.IntakerSubsystem;

public class ManualIntakeOutCommand extends Command {
    private final IntakerSubsystem intakerSubsystem;

    public ManualIntakeOutCommand(IntakerSubsystem intakerSubsystem) {
        this.intakerSubsystem = intakerSubsystem;
        //addRequirements(intakerSubsystem);
    }

    @Override
    public void execute() {
        intakerSubsystem.getIntakerMotor()
                .setVoltage(-Constants.IntakerConstants.intakeVoltage.magnitude());
    }

    @Override
    public void end(boolean interrupted) {
        intakerSubsystem.getIntakerMotor().setVoltage(0);
    }
}
