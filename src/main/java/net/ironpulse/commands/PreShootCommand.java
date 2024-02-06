package net.ironpulse.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

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
        indicatorSubsystem
                .setPattern(IndicatorIO.Patterns.SHOOTING);
    }

    @Override
    public void execute() {
        var targetOptional = Limelight.getTarget();
        if (targetOptional.isEmpty()) {
            shooterSubsystem.getIo().setShooterVoltage(defaultShootVoltage);
            return;
        }

        var target = targetOptional.get();
        var distance = target
                .targetPoseCameraSpace()
                .getTranslation()
                .getDistance(new Translation3d());
        if (distance >= shortShootMaxDistance.magnitude()) {
            shooterSubsystem.getIo().setShooterVoltage(farShootVoltage);
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
