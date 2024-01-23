package net.ironpulse.commands.autos;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.ShooterSubsystem;

public class AutoAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    private final Timer timer = new Timer();

    public AutoAimingCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        timer.start();
        if (Limelight.getTarget().isEmpty()) return;
        var target = Limelight.getTarget().get();
        shooterSubsystem.getDeployMotor().setControl(
                new MotionMagicVoltage(Units.degreesToRotations(90 - target.position().getY())));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }
}
