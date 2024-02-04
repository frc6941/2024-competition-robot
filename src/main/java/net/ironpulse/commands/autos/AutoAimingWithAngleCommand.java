package net.ironpulse.commands.autos;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.ShooterSubsystem;

public class AutoAimingWithAngleCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private Measure<Angle> deployAngle;
    private final Timer timer = new Timer();

    public AutoAimingWithAngleCommand(ShooterSubsystem shooterSubsystem, Measure<Angle> angle) {
        this.shooterSubsystem = shooterSubsystem;
        deployAngle = angle;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        shooterSubsystem.getArmMotor().setControl(
                new MotionMagicVoltage(deployAngle.in(Rotations)));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }
}
