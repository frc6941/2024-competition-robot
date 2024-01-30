package net.ironpulse.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.Rotations;

/**
 * This command will auto adjusting the swerve y offset to make the shooter facing
 * the Amp center and deploy the shooter to desired angle.
 * <p>
 * End condition: When the robot is at IDLE state.
 */
public class AmpAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public AmpAimingCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.getArmMotor().setControl(
                new MotionMagicVoltage(Constants.ShooterConstants.ampDeployAngle.in(Rotations)));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getArmMotor()
                .setControl(new MotionMagicVoltage(0).withSlot(1));
    }
}
