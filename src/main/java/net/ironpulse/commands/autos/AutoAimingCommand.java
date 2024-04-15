package net.ironpulse.commands.autos;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.drivers.LimelightHelpers;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static net.ironpulse.Constants.Logger.debug;
import static net.ironpulse.utils.Utils.autoIntaking;
import static net.ironpulse.utils.Utils.blind;

public class AutoAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    private final Timer timer = new Timer();
    private Measure<Angle> defaultAngle = Degrees.of(20);

    public AutoAimingCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        debug("AutoAimingCommand", "start");
        timer.restart();
        defaultAngle = Degrees.of(20);
    }

    @Override
    public void execute() {
        // if not (indexer beam break on and not intaker beam break on): intaking, do not aim
        if (autoIntaking || blind) {
            shooterSubsystem.getIo().setArmPosition(Radians.zero());
            return;
        }
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
    }

    @Override
    public void end(boolean interrupted) {
        debug("AutoAimingCommand", "end; elapsed=" + timer.get());
        shooterSubsystem.getIo().setArmPosition(Radians.zero());
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(15);
    }
}
