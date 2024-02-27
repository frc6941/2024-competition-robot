package net.ironpulse.commands.climb;


import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.swerve.FieldCentricTargetHeading;
import net.ironpulse.subsystems.swerve.SwerveSubsystem;

import static net.ironpulse.Constants.SwerveConstants.*;

public class ClimbDriveForwardCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private int trust;
    private final FieldCentricTargetHeading drive = new FieldCentricTargetHeading()
            .withDeadband(maxSpeed.magnitude() * 0.1)
            .withRotationalDeadband(0)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    public ClimbDriveForwardCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.trust = 0;
        drive.HeadingController.setPID(headingGains.kP, headingGains.kI, headingGains.kD);
    }

    @Override
    public void execute() {
        swerveSubsystem.applyRequest(() ->
                drive
                        .withVelocityY(climbDriveVoltage.magnitude())
        ).execute();
        var targetOptional = Limelight.getTarget();
        if (targetOptional.isEmpty()) return;
        var target = targetOptional.get();
        swerveSubsystem.applyRequest(() ->
                drive
                        .withVelocityY(climbDriveVoltage.magnitude())
                        .withCurrentTx(target.position().getX())
        ).execute();
        if (target.position().getDistance(target.botPoseWPIBlue().toPose2d().getTranslation()) <= 10) {
            trust++;
        }
    }

    @Override
    public boolean isFinished() {
        return trust >= 20;
    }
}
