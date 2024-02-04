package net.ironpulse.commands.autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Rotations;
import static net.ironpulse.Constants.ShooterConstants.speakerArmOffset;

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
        Limelight.getTarget()
                .ifPresent(target -> shooterSubsystem.getIo().setArmPosition(
                        Rotations.of(Units.degreesToRotations(90 -
                                target.position().getY() - speakerArmOffset.magnitude()))
                ));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }
}
