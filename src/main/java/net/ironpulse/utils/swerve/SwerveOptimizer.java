package net.ironpulse.utils.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static net.ironpulse.Constants.SwerveConstants.maxLinearSpeed;

// From 1678
public class SwerveOptimizer {
    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to include placing
     * in appropriate scope for CTRE onboard control (Also is needed for REV).
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            if (delta > 90) {
                targetAngle -= 180;
            } else {
                targetAngle += 180;
            }
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    public static BetterSwerveModuleState optimize(BetterSwerveModuleState desiredState, Rotation2d currentAngle, double secondOrderOffsetDegrees) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees() + secondOrderOffsetDegrees);
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            if (delta > 90) {
                targetAngle -= 180;
            } else {
                targetAngle += 180;
            }
        }
        return new BetterSwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle), desiredState.omegaRadPerSecond);
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    public static void antiJitter(BetterSwerveModuleState moduleState, BetterSwerveModuleState lastModuleState) {
        if (Math.abs(moduleState.speedMetersPerSecond) <= (maxLinearSpeed.magnitude() * 0.01)) {
            moduleState.angle = lastModuleState.angle;
            moduleState.omegaRadPerSecond = lastModuleState.omegaRadPerSecond;
        }
    }
}