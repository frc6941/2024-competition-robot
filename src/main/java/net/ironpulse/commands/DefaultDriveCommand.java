package net.ironpulse.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.swerve.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import static net.ironpulse.Constants.SwerveConstants.DEADBAND;

public class DefaultDriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier omegaSupplier;

    public DefaultDriveCommand(
            SwerveSubsystem swerveSubsystem,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        // Apply deadband
        double linearMagnitude =
                MathUtil.applyDeadband(
                        Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
        Rotation2d linearDirection =
                new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

        // Square values
        linearMagnitude = linearMagnitude * linearMagnitude;
        omega = Math.copySign(omega * omega, omega);

        // Calculate new linear velocity
        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();

        // Convert to field relative speeds & send command
        boolean isFlipped =
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        swerveSubsystem.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * swerveSubsystem.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * swerveSubsystem.getMaxLinearSpeedMetersPerSec(),
                        omega * swerveSubsystem.getMaxAngularSpeedRadPerSec(),
                        isFlipped
                                ? swerveSubsystem.getRotation().plus(new Rotation2d(Math.PI))
                                : swerveSubsystem.getRotation()));
    }
}
