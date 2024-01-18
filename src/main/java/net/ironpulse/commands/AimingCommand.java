package net.ironpulse.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.maths.Compare;
import net.ironpulse.subsystems.SwerveSubsystem;

public class AimingCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;

    public AimingCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        Limelight.getTarget().ifPresent(target -> swerveSubsystem.applyRequest(() ->
                new SwerveRequest.RobotCentric()
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withVelocityY(target.position().getY() < 0 ? 0.5 : -0.5)
        ).execute());
    }

    @Override
    public boolean isFinished() {
        return (Limelight.getTarget().isEmpty()) ||
                new Compare(Limelight.getTarget().get().position().getX(), 0)
                        .epsilonEqual(1);
    }
}
