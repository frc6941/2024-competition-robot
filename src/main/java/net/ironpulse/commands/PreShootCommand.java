package net.ironpulse.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import static net.ironpulse.Constants.ShooterConstants.*;

public class PreShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private Measure<Voltage> defaultVoltage = shortShootVoltage;

    public PreShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        defaultVoltage = shortShootVoltage;
    }

    @Override
    public void execute() {
        var targetOptional = Limelight.getTarget();
        if (targetOptional.isEmpty()) {
            shooterSubsystem.getIo().setArmPosition(Degrees.of(20));
            shooterSubsystem.getIo().setShooterVoltage(defaultVoltage);
            return;
        }

        var target = targetOptional.get();
        var distance = target
                .targetPoseCameraSpace()
                .getTranslation()
                .getDistance(new Translation3d());
        if (distance >= shortShootMaxDistance.magnitude() + 0.1) {
            defaultVoltage = farShootVoltage;
            shooterSubsystem.getIo().setShooterVoltage(farShootVoltage);
            return;
        }
        if (shortShootMaxDistance.magnitude() - 0.1 < distance && distance < shortShootMaxDistance.magnitude() + 0.1) {
            // looks advanced! do not touch!
//            shooterSubsystem.getIo().setShooterVoltage(Volts.of(
//                    ((distance - shortShootMaxDistance.magnitude() + 0.1) / 0.2) * (farShootVoltage.magnitude() - shortShootVoltage.magnitude()))
//            );
            // Basic math, Watson.
            // Method to derive:
            // delta (farShoot-shortShoot) / delta distance => k
            // substitute one point in => b
            defaultVoltage = Volts.of(
                    15 * distance - 29.5
            ).negate();
            shooterSubsystem.getIo().setShooterVoltage(Volts.of(
                    15 * distance - 29.5
            ).negate());
            return;
        }
        defaultVoltage = shortShootVoltage;
        shooterSubsystem.getIo().setShooterVoltage(shortShootVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setShooterVoltage(Constants.ShooterConstants.shooterConstantVoltage);
    }
}
