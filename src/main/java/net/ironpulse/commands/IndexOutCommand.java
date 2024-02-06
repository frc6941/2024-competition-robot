package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class IndexOutCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;

    public IndexOutCommand(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
    }

    @Override
    public void execute() {
        indexerSubsystem.getIo()
                .setIndexVoltage(Constants.IndexerConstants.indexVoltage.mutableCopy().negate());
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.getIo().setIndexVoltage(Volts.of(0));
    }
}
