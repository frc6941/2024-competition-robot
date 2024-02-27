package net.ironpulse.commands.climb;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

/**
 * Climbs forever without stopping.
 * Used in endgame.
 */
public class ClimbEndgameCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;

    public ClimbEndgameCommand(
            ShooterSubsystem shooterSubsystem,
            IndicatorSubsystem indicatorSubsystem
    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
    }


    @Override
    public void initialize() {
        shooterSubsystem.getIo().setArmBrakeMode(true);
        indicatorSubsystem.setPattern(IndicatorIO.Patterns.CLIMBING);
    }

    @Override
    public void execute() {
        Measure<Voltage> pullVoltage;
        pullVoltage = Constants.ShooterConstants.pullVoltage.negate();
        if (shooterSubsystem.getInputs().pullerTorqueCurrent.gt(Amps.of((50)))) {
            pullVoltage = Volts.zero();
        }
        shooterSubsystem.getIo().setPullerVoltage(pullVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmBrakeMode(false);
    }
}
