package net.ironpulse.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;
import net.ironpulse.subsystems.swerve.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static net.ironpulse.Constants.ShooterConstants.speakerArmOffset;
import static net.ironpulse.Constants.SwerveConstants.DEADBAND;

public class SpeakerAimingCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final PIDController headingPID = new PIDController(0.1, 0, 0);

    public SpeakerAimingCommand(
            SwerveSubsystem swerveSubsystem,
            ShooterSubsystem shooterSubsystem,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
    }

    @Override
    public void execute() {
        var targetOptional = Limelight.getTarget();
        if (targetOptional.isEmpty()) return;
        var target = targetOptional.get();
        if (Math.abs(
                90 - target.position().getY() + speakerArmOffset.magnitude() -
                        shooterSubsystem.getInputs().armPosition.in(Degrees)) >= 1) {
            shooterSubsystem
                    .getIo()
                    .setArmPosition(
                            Radians.of(
                                    Units.degreesToRadians(90 -
                                            target.position().getY() +
                                            speakerArmOffset.magnitude()))
                    );
        }

        double linearMagnitude =
                MathUtil.applyDeadband(
                        Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
        Rotation2d linearDirection =
                new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        double omega = MathUtil.applyDeadband(headingPID.calculate(target.position().getX(),
                0), DEADBAND);

        linearMagnitude = Math.pow(linearMagnitude, 2);
        omega = Math.copySign(omega * omega, omega);

        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();

        boolean isFlipped = DriverStation
                .getAlliance()
                .filter(alliance -> alliance == DriverStation.Alliance.Red)
                .isPresent();
        swerveSubsystem.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * swerveSubsystem.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * swerveSubsystem.getMaxLinearSpeedMetersPerSec(),
                        omega,
                        isFlipped
                                ? swerveSubsystem.getRotation().plus(new Rotation2d(Math.PI))
                                : swerveSubsystem.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmPosition(Radians.of(0));
    }
}
