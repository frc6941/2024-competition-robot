package net.ironpulse.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.BeamBreakSubsystem;
import net.ironpulse.subsystems.IndexerSubsystem;
import net.ironpulse.subsystems.IntakerSubsystem;

public class AutoIntakeCommand extends Command {
    private final IntakerSubsystem intakerSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;

    public AutoIntakeCommand(
            IntakerSubsystem intakerSubsystem,
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem
    ) {
        this.intakerSubsystem = intakerSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
        addRequirements(intakerSubsystem, indexerSubsystem, beamBreakSubsystem);
    }

    @Override
    public void execute() {
        intakerSubsystem.getIntakerMotor()
                .setVoltage(Constants.IntakerConstants.intakeVoltage.magnitude());
        indexerSubsystem.getIndexerMotor()
                .setVoltage(Constants.IndexerConstants.indexVoltage.magnitude());
    }

    @Override
    public void end(boolean interrupted) {
        intakerSubsystem.getIntakerMotor().setVoltage(0);
        indexerSubsystem.getIndexerMotor().setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return beamBreakSubsystem.getIndexerBeamBreak().get() &&
                !beamBreakSubsystem.getIntakerBeamBreak().get();
    }
}
