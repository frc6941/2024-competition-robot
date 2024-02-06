package net.ironpulse.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static net.ironpulse.Constants.IndexerConstants.indexVoltage;
import static net.ironpulse.Constants.ShooterConstants.shooterConstantVoltage;

public class AutoDeliverNoteCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    private final Timer timer = new Timer();

    public AutoDeliverNoteCommand(IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
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
        shooterSubsystem.getIo().setArmPosition(Radians.zero());
        shooterSubsystem.getIo().setShooterVoltage(shooterConstantVoltage);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.5);
    }
}