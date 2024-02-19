package net.ironpulse.commands.autos;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static net.ironpulse.Constants.Logger.debug;
import static net.ironpulse.Constants.ShooterConstants.*;

public class AutoAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    private final Timer timer = new Timer();

    public AutoAimingCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        var targetOptional = Limelight.getTarget();
        var offset = speakerArmOffset.magnitude();
        if (targetOptional.isEmpty()) return;
        var target = targetOptional.get();
        var distance = target
                .targetPoseCameraSpace()
                .getTranslation()
                .getDistance(new Translation3d());

        if (distance >= shortShootMaxDistance.magnitude()) {
            offset = speakerArmOffsetFar.magnitude();
            debug("Shooter:", "far shoot: offset = " + offset);
        } else if (distance >= 2.1) {
            offset = speakerArmOffset.magnitude() +
                    (distance - 2.1) / (shortShootMaxDistance.magnitude() - 2.1) *
                            (speakerArmOffsetFar.magnitude() -
                                    speakerArmOffset.magnitude());
            debug("Shooter:", "far but not too far: offset = " + offset);
        } else if (distance >= 1.3) {
            offset = speakerArmOffsetNear.magnitude() +
                    (distance - 1.3) / (2.1 - 1.3) *
                            (speakerArmOffset.magnitude() -
                                    speakerArmOffsetNear.magnitude());
            debug("Shooter:", "near but not too near: offset = " + offset);
        } else {
            offset = speakerArmOffsetNear.magnitude();
            debug("Shooter:", "near shoot: offset = " + offset);
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
    }

    @Override
    public void end(boolean isInterrupted) {
        shooterSubsystem.getIo().setArmPosition(Radians.zero());
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }
}
