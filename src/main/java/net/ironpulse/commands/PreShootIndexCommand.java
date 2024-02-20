package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;

import static edu.wpi.first.units.Units.Volts;
import static net.ironpulse.Constants.IndexerConstants.indexShootVoltage;

public class PreShootIndexCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;


    public PreShootIndexCommand(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
    }

    @Override
    public void execute() {
        indexerSubsystem.getIo().setIndexVoltage(indexShootVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.getIo()
                .setIndexVoltage(Volts.zero());
    }
}
