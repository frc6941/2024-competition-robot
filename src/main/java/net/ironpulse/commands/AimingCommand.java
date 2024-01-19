package net.ironpulse.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.maths.Compare;
import net.ironpulse.subsystems.SwerveSubsystem;

import static net.ironpulse.Constants.SwerveConstants.maxAngularRate;
import static net.ironpulse.Constants.SwerveConstants.maxSpeed;

public class AimingCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;

    public AimingCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        Limelight.getTarget().ifPresent(target -> swerveSubsystem.applyRequest(() ->
                new SwerveRequest.FieldCentric()
                        .withDeadband(maxSpeed.magnitude() * 0.1)
                        .withRotationalDeadband(maxAngularRate.magnitude() * 0.1)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withRotationalRate(target.position().getX() < 0 ? -0.8 : 0.8)
        ).execute());
    }

    @Override
    public boolean isFinished() {
        return (Limelight.getTarget().isEmpty()) ||
                new Compare(Limelight.getTarget().get().position().getX(), 0)
                        .epsilonEqual(1);
    }
}
