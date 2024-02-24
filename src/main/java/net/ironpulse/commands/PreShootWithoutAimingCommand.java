package net.ironpulse.commands;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

public class PreShootWithoutAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Measure<Voltage> volts;


    public PreShootWithoutAimingCommand(ShooterSubsystem shooterSubsystem, Measure<Voltage> volts) {
        this.shooterSubsystem = shooterSubsystem;
        this.volts = volts;

    }

    @Override
    public void execute() {
        shooterSubsystem.getIo().setShooterVoltage(volts);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setShooterVoltage(Constants.ShooterConstants.shooterConstantVoltage);
    }
}
