package net.ironpulse.commands.autos;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.ShooterSubsystem;
import net.ironpulse.subsystems.drive.Drive;

public class AutoAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Drive swerveSubsystem;

    private final Timer timer = new Timer();

    public AutoAimingCommand(ShooterSubsystem shooterSubsystem, Drive swerveSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        Limelight.getTarget()
                .ifPresent(target -> {
                            System.out.println("Target -> " + target.position().getY());

                            shooterSubsystem
                                    .getArmMotor()
                                    .setControl(new MotionMagicVoltage(
                                            Units.degreesToRotations(90 - target.position().getY() +
                                                    Constants.ShooterConstants.speakerArmOffset.magnitude())));
                        }
                );
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getArmMotor()
                .setControl(new MotionMagicVoltage(0).withSlot(1));
    }
}
