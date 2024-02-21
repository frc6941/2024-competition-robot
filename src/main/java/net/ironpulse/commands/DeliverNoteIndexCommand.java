package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;
import static net.ironpulse.Constants.ShooterConstants.shooterIndexVoltage;

public class DeliverNoteIndexCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;

    public DeliverNoteIndexCommand(
            ShooterSubsystem shooterSubsystem,
            BeamBreakSubsystem beamBreakSubsystem
    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
    }

    @Override
    public void initialize() {
        shooterSubsystem.getIo().setShooterVoltage(Volts.zero());
    }

    @Override
    public void execute() {
        // Yes, it's weird. Not my idea.
        shooterSubsystem.getIo().setShooterVoltage(shooterIndexVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setShooterVoltage(Constants.ShooterConstants.shooterConstantVoltage);
    }

    @Override
    public boolean isFinished() {
        return !beamBreakSubsystem.getInputs().isIndexerBeamBreakOn &&
                !beamBreakSubsystem.getInputs().isShooterBeamBreakOn;
    }
}
