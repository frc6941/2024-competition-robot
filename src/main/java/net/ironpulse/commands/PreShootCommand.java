package net.ironpulse.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;
import static net.ironpulse.Constants.ShooterConstants.*;

public class PreShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;


    public PreShootCommand(ShooterSubsystem shooterSubsystem,
                           IndicatorSubsystem indicatorSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
    }

    @Override
    public void initialize() {
//        indicatorSubsystem
//                .setPattern(IndicatorIO.Patterns.SHOOTING);
    }

    @Override
    public void execute() {
        var targetOptional = Limelight.getTarget();
        if (targetOptional.isEmpty()) {
            return;
        }

        var target = targetOptional.get();
        var distance = target
                .targetPoseCameraSpace()
                .getTranslation()
                .getDistance(new Translation3d());
        if (distance >= shortShootMaxDistance.magnitude() + 0.1) {
            shooterSubsystem.getIo().setShooterVoltage(farShootVoltage);
            return;
        }
        if (shortShootMaxDistance.magnitude() + 0.1 < distance && distance >= shortShootMaxDistance.magnitude() - 0.1) {
            // looks advanced! do not touch!
            shooterSubsystem.getIo().setShooterVoltage(Volts.of(
                    ((distance - shortShootMaxDistance.magnitude() + 0.1) / 0.2) * (farShootVoltage.magnitude() - shortShootVoltage.magnitude()))
            );
            return;
        }
        shooterSubsystem.getIo().setShooterVoltage(shortShootVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setShooterVoltage(Constants.ShooterConstants.shooterConstantVoltage);
        if (!interrupted) return;
        indicatorSubsystem.resetToLastPattern();
    }
}
