package net.ironpulse.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.Rotations;

public class ParallelAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ParallelAimingCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.getArmMotor().setControl(
                new MotionMagicVoltage(Constants.ShooterConstants.parallelDeployAngle.in(Rotations)));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getArmMotor()
                .setControl(new MotionMagicVoltage(0).withSlot(1));
    }
}
