package net.ironpulse.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static net.ironpulse.utils.Utils.armReachedClimb;

public class ClimbShooterUpCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Timer timer = new Timer();

    public ClimbShooterUpCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        shooterSubsystem.getIo()
                .setPullerBrakeMode(true);
        timer.restart();
    }

    @Override
    public void execute() {
        var voltage = Constants.ShooterConstants.shooterUpDownVoltage.mutableCopy().negate();
        if (shooterSubsystem.getInputs().armPosition.minus(Radians.of(2.52)).gt(Radians.of(0.04))) {
            voltage = Volts.zero();
            armReachedClimb = true;
        }
        if (!timer.hasElapsed(0.3)) {
            shooterSubsystem.getIo()
                    .setPullerVoltage(Constants.ShooterConstants.pullVoltage);
        } else {
            shooterSubsystem.getIo()
                    .setPullerVoltage(Volts.zero());
            shooterSubsystem.getIo()
                    .setArmVoltage(voltage);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setPullerBrakeMode(false);
        shooterSubsystem.getIo().setArmVoltage(Volts.zero());
    }

    @Override
    public boolean isFinished() {
        return armReachedClimb;
    }
}
