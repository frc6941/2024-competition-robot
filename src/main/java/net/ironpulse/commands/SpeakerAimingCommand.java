package net.ironpulse.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.RobotContainer;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.ShooterSubsystem;
import net.ironpulse.subsystems.SwerveSubsystem;
import net.ironpulse.swerve.FieldCentricTargetHeading;

import static net.ironpulse.Constants.SwerveConstants.*;

public class SpeakerAimingCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final RobotContainer robotContainer;
    private final FieldCentricTargetHeading drive = new FieldCentricTargetHeading()
            .withDeadband(maxSpeed.magnitude() * 0.1)
            .withRotationalDeadband(maxAngularRate.magnitude() * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    public SpeakerAimingCommand(
            RobotContainer robotContainer,
            ShooterSubsystem shooterSubsystem,
            SwerveSubsystem swerveSubsystem
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.robotContainer = robotContainer;
        drive.HeadingController.setPID(headingGains.kP, headingGains.kI, headingGains.kD);
    }

    @Override
    public void execute() {
        var targetOptional = Limelight.getTarget();
        if (targetOptional.isEmpty()) return;
        var target = targetOptional.get();
        swerveSubsystem.applyRequest(() ->
                drive
                        .withVelocityX(-robotContainer.getDriverController().getLeftY()
                                * maxSpeed.magnitude())
                        .withVelocityY(-robotContainer.getDriverController().getLeftX()
                                * maxSpeed.magnitude())
                        .withCurrentTx(target.position().getX())
        ).execute();
        shooterSubsystem.getArmMotor()
                .setControl(new MotionMagicVoltage(
                        Units.degreesToRotations(90 - target.position().getY() +
                                Constants.ShooterConstants.shooterDeployOffset.magnitude())));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getArmMotor()
                .setControl(new MotionMagicVoltage(0).withSlot(1));
    }

    @Override
    public boolean isFinished() {
        return !robotContainer.getBeamBreakSubsystem().getIndexerBeamBreak().get() &&
                !robotContainer.getBeamBreakSubsystem().getShooterLeftBeamBreak().get();
    }
}
