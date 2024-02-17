package net.ironpulse.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;
import net.ironpulse.subsystems.swerve.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class SpeakerAimingCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final PIDController headingPID = new PIDController(0.1, 0, 0);

    public SpeakerAimingCommand(
            SwerveSubsystem swerveSubsystem,
            ShooterSubsystem shooterSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
    }

    @Override
    public void execute() {
        this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
        var targetOptional = Limelight.getTarget();
        var offset = Constants.ShooterConstants.speakerArmOffset.magnitude();
        if (targetOptional.isEmpty()) return;
        this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);
        var target = targetOptional.get();
        var distance = target.
                targetPoseCameraSpace().
                getTranslation().
                getDistance(new Translation3d());
        if (distance >= Constants.ShooterConstants.shortShootMaxDistance.magnitude()) {
            offset = Constants.ShooterConstants.speakerArmOffsetFar.magnitude();
            System.out.println("far shoot: offset = " + offset);
        } else if (distance >= 2.1) {
            offset = Constants.ShooterConstants.speakerArmOffset.magnitude() +
                    (distance - 2.1) / (Constants.ShooterConstants.shortShootMaxDistance.magnitude() - 2.1) *
                            (Constants.ShooterConstants.speakerArmOffsetFar.magnitude() -
                                    Constants.ShooterConstants.speakerArmOffset.magnitude());
            System.out.println("far but not too far: offset = " + offset);
        } else if (distance >= 1.3) {
            offset = Constants.ShooterConstants.speakerArmOffsetNear.magnitude() +
                    (distance - 1.3) / (2.1 - 1.3) *
                            (Constants.ShooterConstants.speakerArmOffset.magnitude() -
                                    Constants.ShooterConstants.speakerArmOffsetNear.magnitude());
            System.out.println("near but not too near: offset = " + offset);
        } else {
            offset = Constants.ShooterConstants.speakerArmOffsetNear.magnitude();
            System.out.println("near shoot: offset = " + offset);
        }

        if (Math.abs(
                90 - target.position().getY() + offset -
                        shooterSubsystem.getInputs().armPosition.in(Degrees)) >= 3) {
            shooterSubsystem
                    .getIo()
                    .setArmPosition(
                            Radians.of(
                                    Units.degreesToRadians(90 -
                                            target.position().getY() +
                                            offset))
                    );
        }

//        double linearMagnitude =
//                MathUtil.applyDeadband(
//                        Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
//        Rotation2d linearDirection =
//                new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
//        double omega = MathUtil.applyDeadband(headingPID.calculate(target.position().getX(),
//                0), DEADBAND);
//
//        linearMagnitude = Math.pow(linearMagnitude, 2);
//        omega = Math.copySign(omega * omega, omega);
//
//        Translation2d linearVelocity =
//                new Pose2d(new Translation2d(), linearDirection)
//                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
//                        .getTranslation();
//
//        boolean isFlipped = DriverStation
//                .getAlliance()
//                .filter(alliance -> alliance == DriverStation.Alliance.Red)
//                .isPresent();
//        swerveSubsystem.runVelocity(
//                ChassisSpeeds.fromFieldRelativeSpeeds(
//                        linearVelocity.getX() * swerveSubsystem.getMaxLinearSpeedMetersPerSec(),
//                        linearVelocity.getY() * swerveSubsystem.getMaxLinearSpeedMetersPerSec(),
//                        omega,
//                        isFlipped
//                                ? swerveSubsystem.getRotation().plus(new Rotation2d(Math.PI))
//                                : swerveSubsystem.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmPosition(Radians.zero());
    }
}
