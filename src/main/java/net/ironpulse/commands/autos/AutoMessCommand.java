package net.ironpulse.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.intaker.IntakerSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;
import static net.ironpulse.Constants.IndexerConstants.indexVoltage;
import static net.ironpulse.Constants.IntakerConstants.intakeVoltage;
import static net.ironpulse.Constants.ShooterConstants.shooterConstantVoltage;

public class AutoMessCommand extends Command {
    private final IntakerSubsystem intakerSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    private final Timer timer = new Timer();

    public AutoMessCommand(
            IntakerSubsystem intakerSubsystem,
            IndexerSubsystem indexerSubsystem,
            ShooterSubsystem shooterSubsystem
    ) {
        this.intakerSubsystem = intakerSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        intakerSubsystem.getIo().setIntakeVoltage(intakeVoltage);
        indexerSubsystem.getIo().setIndexVoltage(indexVoltage);
        shooterSubsystem.getIo().setShooterVoltage(shooterConstantVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setShooterVoltage(Volts.zero());
        intakerSubsystem.getIo().setIntakeVoltage(Volts.zero());
        indexerSubsystem.getIo().setIndexVoltage(Volts.zero());
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(15);
    }
}
