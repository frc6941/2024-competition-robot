package net.ironpulse.commands.manuals;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.ShooterSubsystem;

public class ManualShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ManualShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.getShootMotorLeft()
                .setVoltage(Constants.IntakerConstants.intakeVoltage.magnitude());
        shooterSubsystem.getShootMotorRight()
                .setVoltage(Constants.IntakerConstants.intakeVoltage.magnitude());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getShootMotorLeft().setVoltage(0);
        shooterSubsystem.getShootMotorRight().setVoltage(0);
    }
}
