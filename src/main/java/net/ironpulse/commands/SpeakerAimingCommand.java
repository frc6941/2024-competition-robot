package net.ironpulse.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import net.ironpulse.Constants;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;
import net.ironpulse.subsystems.swerve.FieldCentricTargetHeading;
import net.ironpulse.subsystems.swerve.SwerveSubsystem;
import net.ironpulse.utils.Utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static net.ironpulse.Constants.Logger.debug;
import static net.ironpulse.Constants.ShooterConstants.shootMaxDistance;
import static net.ironpulse.Constants.SwerveConstants.*;

public class SpeakerAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController driverController;
    private final FieldCentricTargetHeading drive = new FieldCentricTargetHeading()
            .withDeadband(maxSpeed.magnitude() * 0.1)
            .withRotationalDeadband(0)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    public SpeakerAimingCommand(
            ShooterSubsystem shooterSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            SwerveSubsystem swerveSubsystem,
            CommandXboxController driverController
    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.driverController = driverController;
        drive.HeadingController.setPID(headingGains.kP, headingGains.kI, headingGains.kD);
    }

    @Override
    public void execute() {
        this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
        var targetOptional = Limelight.getTarget();
        var offset = Constants.ShooterConstants.speakerArmOffset.magnitude();
        if (targetOptional.isEmpty()) return;
        var target = targetOptional.get();
        var distance = target.targetPoseCameraSpace().getTranslation().getDistance(new Translation3d());
        var angle = Units.radiansToDegrees(target.targetPoseCameraSpace().getRotation().getAngle());
        debug("Shooter:",
                "targetId => " + target.tagId() +
                        " distance => " + distance + " angle => " + angle);
        this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);
        if (distance >= shootMaxDistance.magnitude()) {
            offset = Constants.ShooterConstants.speakerArmOffsetMax.magnitude();
            debug("Shooter:", "max shoot: offset = " + offset);
        } else if (distance >= 2.7) {
            offset = Constants.ShooterConstants.speakerArmOffsetFar.magnitude() +
                    (distance - 2.7) / (shootMaxDistance.magnitude() - 2.7) *
                            (Constants.ShooterConstants.speakerArmOffsetMax.magnitude() -
                                    Constants.ShooterConstants.speakerArmOffsetFar.magnitude());
            debug("Shooter:", "far: offset = " + offset);
        } else if (distance >= 2.1) {
            offset = Constants.ShooterConstants.speakerArmOffset.magnitude() +
                    (distance - 2.1) / (Constants.ShooterConstants.shortShootMaxDistance.magnitude() - 2.1) *
                            (Constants.ShooterConstants.speakerArmOffsetFar.magnitude() -
                                    Constants.ShooterConstants.speakerArmOffset.magnitude());
            debug("Shooter:", "far but not too far: offset = " + offset);
        } else if (distance >= 1.3) {
            offset = Constants.ShooterConstants.speakerArmOffsetNear.magnitude() +
                    (distance - 1.3) / (2.1 - 1.3) *
                            (Constants.ShooterConstants.speakerArmOffset.magnitude() -
                                    Constants.ShooterConstants.speakerArmOffsetNear.magnitude());
            debug("Shooter:", "near but not too near: offset = " + offset);
        } else {
            offset = Constants.ShooterConstants.speakerArmOffsetNear.magnitude();
            debug("Shooter:", "near shoot: offset = " + offset);
        }

        if (angle >= 40) {
            offset = offset - (angle - 40) / 10 * 3;
        }


        if (Math.abs(
                offset -
                        shooterSubsystem.getInputs().armPosition.in(Degrees)) >= 2.0) {
            shooterSubsystem
                    .getIo()
                    .setArmPosition(
                            Radians.of(
                                    Units.degreesToRadians(offset))
                    );
        }

        swerveSubsystem.applyRequest(() ->
                drive
                        .withVelocityX(Utils.sign(-driverController.getLeftY())
                                * xLimiter.calculate(Math.abs(driverController.getLeftY()))
                                * maxSpeed.magnitude())
                        .withVelocityY(
                                Utils.sign(-driverController.getLeftX()) * maxSpeed.magnitude()
                                        * yLimiter.calculate(Math.abs(driverController.getLeftX()))
                        )
                        .withCurrentTx(target.position().getX() * 1.6)
        ).execute();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmPosition(Radians.zero());
    }
}
