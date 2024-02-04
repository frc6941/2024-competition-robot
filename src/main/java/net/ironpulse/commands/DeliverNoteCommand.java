package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.indicator.IndicatorIO;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static net.ironpulse.Constants.IndexerConstants.indexVoltage;

public class DeliverNoteCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;
    private final RobotContainer robotContainer;

    public DeliverNoteCommand(
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            RobotContainer robotContainer
    ) {
        this.indexerSubsystem = indexerSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
        this.robotContainer = robotContainer;
    }

    @Override
    public void execute() {
        indexerSubsystem.getIo().setIndexVoltage(indexVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.getIo().setIndexVoltage(Volts.of(0));
        if (interrupted) return;
        robotContainer.getIndicatorSubsystem()
                .setPattern(IndicatorIO.Patterns.FINISH_SHOOT);
        Commands.runOnce(() -> new RumbleCommand(robotContainer.getDriverController().getHID(), Seconds.of(0.5)));
    }

    @Override
    public boolean isFinished() {
        return !beamBreakSubsystem.getInputs().isIndexerBeamBreakOn &&
                !beamBreakSubsystem.getInputs().isShooterBeamBreakOn;
    }
}
