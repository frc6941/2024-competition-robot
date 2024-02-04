package net.ironpulse.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;

import static edu.wpi.first.units.Units.Volts;
import static net.ironpulse.Constants.IndexerConstants.indexVoltage;

public class AutoDeliverNoteCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;

    private final Timer timer = new Timer();

    public AutoDeliverNoteCommand(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        indexerSubsystem.getIo().setIndexVoltage(indexVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.getIo().setIndexVoltage(Volts.zero());
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.5);
    }
}
