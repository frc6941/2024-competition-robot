package net.ironpulse.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.RobotContainer;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.maths.Angle;
import net.ironpulse.subsystems.ShooterSubsystem;
import net.ironpulse.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import static net.ironpulse.Constants.SwerveConstants.*;
import static net.ironpulse.state.StateMachine.Actions;

public class SpeakerAimingCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final RobotContainer robotContainer;
    private final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed.magnitude() * 0.1)
            .withRotationalDeadband(maxAngularRate.magnitude() * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    private final Supplier<Boolean> confirmation;

    public SpeakerAimingCommand(
            RobotContainer robotContainer,
            ShooterSubsystem shooterSubsystem,
            SwerveSubsystem swerveSubsystem,
            Supplier<Boolean> confirmation
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.robotContainer = robotContainer;
        this.confirmation = confirmation;
        drive.HeadingController.setPID(headingGains.kP, headingGains.kI, headingGains.kD);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        robotContainer.getGlobalStateMachine().transfer(Actions.SHOOT);
        if (Limelight.getTarget().isEmpty()) return;
        var target = Limelight.getTarget().get();
        swerveSubsystem.applyRequest(() ->
                drive
                        .withVelocityX(-robotContainer.getDriverController().getLeftY()
                                * maxSpeed.magnitude())
                        .withVelocityY(-robotContainer.getDriverController().getLeftX()
                                * maxSpeed.magnitude())
                        .withTargetDirection(
                                Rotation2d.fromDegrees(
                                        Angle.continuousToPositive360(
                                                Angle.continuousToPositive360(
                                                        swerveSubsystem.getPigeon2().getAngle()) + 180)
                                                + target.position().getX()))
        ).execute();
        shooterSubsystem.getDeployMotor()
                .setControl(new MotionMagicVoltage(
                        Units.degreesToRotations(90 - target.position().getY())));
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) return;
        robotContainer.getGlobalStateMachine().transfer(Actions.INTERRUPT_SHOOT);
    }

    @Override
    public boolean isFinished() {
        return confirmation.get();
    }
}
