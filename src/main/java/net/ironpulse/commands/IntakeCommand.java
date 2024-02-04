package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import net.ironpulse.Constants;
import net.ironpulse.RobotContainer;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.intaker.IntakerSubsystem;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class IntakeCommand extends Command {
    private final IntakerSubsystem intakerSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;
    private final RobotContainer robotContainer;

    public IntakeCommand(
            IntakerSubsystem intakerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            RobotContainer robotContainer
    ) {
        this.intakerSubsystem = intakerSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
        this.robotContainer = robotContainer;
    }

    @Override
    public void execute() {
        if (isFinished()) return;
        intakerSubsystem.getIo()
                .setIntakeVoltage(Constants.IntakerConstants.intakeVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        intakerSubsystem.getIo()
                .setIntakeVoltage(Volts.of(0));
        if (interrupted) return;
        robotContainer.getIndicatorSubsystem()
                .setPattern(IndicatorIO.Patterns.FINISH_INTAKE);
        Commands.runOnce(() -> new RumbleCommand(robotContainer.getDriverController().getHID(), Seconds.of(0.5)));
    }


    @Override
    public boolean isFinished() {
        return beamBreakSubsystem.getInputs().isIndexerBeamBreakOn &&
                !beamBreakSubsystem.getInputs().isIntakerBeamBreakOn;
    }
}
