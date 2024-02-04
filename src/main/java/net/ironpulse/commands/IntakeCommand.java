package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.intaker.IntakerSubsystem;

public class IntakeCommand extends Command {
    private final IntakerSubsystem intakerSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;

    public IntakeCommand(
            IntakerSubsystem intakerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem
    ) {
        this.intakerSubsystem = intakerSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
    }

    @Override
    public void execute() {
        if (isFinished()) return;
        intakerSubsystem.getIo()
                .setIntakeVoltage(Constants.IntakerConstants.intakeVoltage);
    }

    @Override
    public boolean isFinished() {
        return beamBreakSubsystem.getInputs().isIndexerBeamBreakOn &&
                !beamBreakSubsystem.getInputs().isIntakerBeamBreakOn;
    }
}
