package net.ironpulse.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.IntakerSubsystem;

public class ResetIntakerCommand extends Command {
    private final IntakerSubsystem intakerSubsystem;
    private final Timer timer = new Timer();

    public ResetIntakerCommand(IntakerSubsystem intakerSubsystem) {
        this.intakerSubsystem = intakerSubsystem;
    }

    @Override
    public void execute() {
        intakerSubsystem.getIntakerMotor()
                .setVoltage(-Constants.IntakerConstants.intakeVoltage.magnitude());
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
        intakerSubsystem.getIntakerMotor().setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }
}
