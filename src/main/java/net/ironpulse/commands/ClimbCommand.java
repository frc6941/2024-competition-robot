package net.ironpulse.commands;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class ClimbCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final boolean reverse;

    public ClimbCommand(
            ShooterSubsystem shooterSubsystem,
            boolean reverse
    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.reverse = reverse;
    }


    @Override
    public void initialize() {
        shooterSubsystem.getIo().setArmBrakeMode(true);
    }

    @Override
    public void execute() {
        Measure<Voltage> pullVoltage;
        if (reverse) {
            pullVoltage = Constants.ShooterConstants.pullVoltage.negate();
        } else {
            pullVoltage = Constants.ShooterConstants.pullVoltage;
        }
        shooterSubsystem.getIo().setPullerVoltage(pullVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmBrakeMode(false);
        shooterSubsystem.getIo()
                .setPullerVoltage(Volts.zero());
    }
}
