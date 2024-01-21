package net.ironpulse.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import net.ironpulse.Constants;
import net.ironpulse.RobotContainer;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.maths.Compare;
import net.ironpulse.subsystems.ShooterSubsystem;
import net.ironpulse.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import static net.ironpulse.Constants.SwerveConstants.maxAngularRate;
import static net.ironpulse.Constants.SwerveConstants.maxSpeed;
import static net.ironpulse.state.StateMachine.Actions;

public class SpeakerAimingCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final RobotContainer robotContainer;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed.magnitude() * 0.1)
            .withRotationalDeadband(maxAngularRate.magnitude() * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
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
        addRequirements(swerveSubsystem, shooterSubsystem);
    }

    @Override
    public void execute() {
        robotContainer.getGlobalState().transfer(Actions.SHOOT);
        Limelight.getTarget().ifPresent(target ->
                CommandScheduler.getInstance().schedule(
                        swerveSubsystem.applyRequest(() ->
                                drive
                                        .withVelocityX(-robotContainer.getDriverController()
                                                .getLeftY() * maxSpeed.magnitude())
                                        .withVelocityY(-robotContainer.getDriverController()
                                                .getLeftX() * maxSpeed.magnitude())
                                        .withRotationalRate(atSetpoint() ? 0 : target.position().getX() < 0 ? -0.8 : 0.8)
                        )
                )
        );
        // TODO Test whether ty is the pitch angle
        Limelight.getTarget().ifPresent(target ->
                shooterSubsystem.getDeployMotor()
                        .setControl(new MotionMagicVoltage(
                                Units.degreesToRotations(90 - target.position().getY() +
                                        Constants.ShooterConstants.speakerAngleOffset.magnitude())))
        );
    }

    private boolean atSetpoint() {
        return Limelight.getTarget().isPresent() && new Compare(
                Limelight.getTarget().get().position().getX(), 0).epsilonEqual(1);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) return;
        robotContainer.getGlobalState().transfer(Actions.INTERRUPT_SHOOT);
    }

    @Override
    public boolean isFinished() {
        return confirmation.get();
    }
}
