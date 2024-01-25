package net.ironpulse.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.IndexerSubsystem;

public class AutoDeliverNoteCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;

    private final Timer timer = new Timer();

    public AutoDeliverNoteCommand(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        timer.start();
        indexerSubsystem.getIndexerMotor()
                .setVoltage(Constants.IndexerConstants.indexVoltage.magnitude());
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.getIndexerMotor().setVoltage(0);
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }
}
