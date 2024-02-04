package net.ironpulse.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.Rotations;

public class ParallelAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private Measure<Angle> deployAngle;

    public ParallelAimingCommand(ShooterSubsystem shooterSubsystem, int angle) {
        this.shooterSubsystem = shooterSubsystem;
        deployAngle= Units.Degrees.of(angle);
    }

    @Override
    public void execute() {
        shooterSubsystem.getArmMotor().setControl(
                new MotionMagicVoltage(deployAngle.in(Rotations)));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getArmMotor()
                .setControl(new MotionMagicVoltage(0).withSlot(1));
    }
}
