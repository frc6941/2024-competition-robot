package net.ironpulse.commands.autos;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static net.ironpulse.Constants.Logger.debug;
import static net.ironpulse.utils.Utils.intaking;

public class AutoAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;

    private final Timer timer = new Timer();

    public AutoAimingCommand(ShooterSubsystem shooterSubsystem, BeamBreakSubsystem beamBreakSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
    }

    @Override
    public void initialize() {
        debug("AutoAimingCommand", "start");
        timer.restart();
    }

    @Override
    public void execute() {
        // if not (indexer beam break on and not intaker beam break on): intaking, do not aim
        if (intaking) {
            shooterSubsystem.getIo().setArmPosition(Radians.zero());
            return;
        }
        var targetOptional = Limelight.getTarget();
        var offset = Constants.ShooterConstants.speakerArmOffset.magnitude();
        if (targetOptional.isEmpty()) return;
        var target = targetOptional.get();
        var distance = target.
                targetPoseCameraSpace().
                getTranslation().
                getDistance(new Translation3d());
        debug("Shooter:", "far => " + Constants.ShooterConstants.speakerArmOffsetFar.magnitude() + " normal => " + Constants.ShooterConstants.speakerArmOffset.magnitude() + " short => " + Constants.ShooterConstants.speakerArmOffsetNear.magnitude());
        if (distance >= Constants.ShooterConstants.shortShootMaxDistance.magnitude()) {
            offset = Constants.ShooterConstants.speakerArmOffsetFar.magnitude();
            debug("Shooter:", "far shoot: offset = " + offset);
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

        if (Math.abs(
                90 - target.position().getY() + offset -
                        shooterSubsystem.getInputs().armPosition.in(Degrees)) >= 0.5) {
            shooterSubsystem
                    .getIo()
                    .setArmPosition(
                            Radians.of(
                                    Units.degreesToRadians(90 -
                                            target.position().getY() +
                                            offset))
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
