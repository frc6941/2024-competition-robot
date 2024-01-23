package net.ironpulse.commands.manuals;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.Rotations;

public class ManualShooterDownCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ManualShooterDownCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.getDeployMotor()
                .setControl(new MotionMagicVoltage(
                        shooterSubsystem.getDeployMotor().getPosition().getValue() -
                                Constants.ShooterConstants.manualAimingStep.in(Rotations)));
    }
}
