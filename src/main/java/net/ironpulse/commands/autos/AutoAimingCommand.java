package net.ironpulse.commands.autos;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

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
//        Limelight.getTarget()
//                .ifPresent(target -> shooterSubsystem.getIo().setArmPosition(
//                        Radians.of(Units.degreesToRadians(90 -
//                                target.position().getY() - speakerArmOffset.magnitude()))
//                ));
        var targetOptional = Limelight.getTarget();
        var offset = Constants.ShooterConstants.speakerArmOffset.magnitude();
        if (targetOptional.isEmpty()) return;
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
    }

    @Override
    public void end(boolean isInterrupted) {
        shooterSubsystem.getIo().setArmPosition(Radians.zero(), 1);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }
}
