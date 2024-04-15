package net.ironpulse.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import net.ironpulse.Constants;
import net.ironpulse.drivers.LimelightHelpers;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;
import net.ironpulse.subsystems.swerve.FieldCentricTargetHeading;
import net.ironpulse.subsystems.swerve.SwerveSubsystem;
import net.ironpulse.utils.Utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static net.ironpulse.Constants.Logger.debug;
import static net.ironpulse.Constants.SwerveConstants.*;

public class SpeakerAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController driverController;
    private Measure<Angle> defaultAngle = Degrees.of(20);
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
    public void initialize() {
        defaultAngle = Degrees.of(20);
    }

    @Override
    public void execute() {
        this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
        var offset = Constants.ShooterConstants.speakerArmOffset.magnitude();
        boolean hasTarget = LimelightHelpers.getTV("limelight");
        if (!hasTarget) {
            shooterSubsystem.getIo().setArmPosition(defaultAngle);
            return;
        }
        Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
        var distance = target.getTranslation().getDistance(new Translation3d());
        var angle = Units.radiansToDegrees(target.getRotation().getAngle());
        if (distance == 0) {
            debug("Shooter:", "wtf?");
            shooterSubsystem.getIo().setArmPosition(defaultAngle);
            return;
        }
        debug("Shooter:",
                " distance => " + distance + " angle => " + angle);
        this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);

        // Calculated using highly-sophisticated software.
        // Do not touch unless you (really) know what you're doing!
//        offset = -297 + 573.9 * distance - 427.3 * Math.pow(distance, 2) + 168.9 * Math.pow(distance, 3) - 33.43 * Math.pow(distance, 4) + 2.593 * Math.pow(distance, 5);
        double A1 = 18.43145;
        double A2 = 67.62172;
        double x0 = 2.07751;
        double p = 5.16297;
        offset = A2 + (A1 - A2) / (1 + Math.pow(distance / x0, p));
        debug("Shooter:", "offset = " + offset);
        if (0 > offset || offset > 180) {
            debug("Shooter:", "wtf?");
            shooterSubsystem.getIo().setArmPosition(defaultAngle);
            return;
        }

        if (Math.abs(
                offset -
                        shooterSubsystem.getInputs().armPosition.in(Degrees)) >= 0.5) {
            defaultAngle = Degrees.of(offset);
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
                        .withCurrentTx(LimelightHelpers.getTX("limelight") * 1.6)
        ).execute();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmPosition(Radians.zero());
    }
}
