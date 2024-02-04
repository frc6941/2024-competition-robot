package net.ironpulse.commands.manuals;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.IndexerSubsystem;

public class ManualIndexInCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;

    public ManualIndexInCommand(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        //addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        indexerSubsystem.getIndexerMotor()
                .setVoltage(Constants.IndexerConstants.indexVoltage.magnitude());
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.getIndexerMotor().setVoltage(0);
    }
}
