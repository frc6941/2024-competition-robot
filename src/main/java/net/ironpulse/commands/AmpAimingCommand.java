package net.ironpulse.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.RobotContainer;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.state.StateMachine;
import net.ironpulse.subsystems.ShooterSubsystem;
import net.ironpulse.subsystems.SwerveSubsystem;
import net.ironpulse.swerve.RobotCentricTargetTx;

import static edu.wpi.first.units.Units.Rotations;
import static net.ironpulse.Constants.SwerveConstants.*;
import static net.ironpulse.state.StateMachine.Actions;

/**
 * This command will auto adjusting the swerve y offset to make the shooter facing
 * the Amp center and deploy the shooter to desired angle.
 * <p>
 * End condition: When the robot is at IDLE state.
 */
public class AmpAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final RobotContainer robotContainer;
    private final RobotCentricTargetTx drive = new RobotCentricTargetTx()
            .withDeadband(maxSpeed.magnitude() * 0.1)
            .withRotationalDeadband(maxAngularRate.magnitude() * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    public AmpAimingCommand(
            ShooterSubsystem shooterSubsystem,
            SwerveSubsystem swerveSubsystem,
            RobotContainer robotContainer
    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.robotContainer = robotContainer;
        drive.TxController.setPID(txGains.kP, txGains.kI, txGains.kD);
        addRequirements(shooterSubsystem, swerveSubsystem);
    }

    @Override
    public void execute() {
        robotContainer.getGlobalStateMachine().transfer(Actions.SHOOT);
        shooterSubsystem.getDeployMotor().setControl(
                new MotionMagicVoltage(Constants.ShooterConstants.ampDeployAngle.in(Rotations)));
        if (Limelight.getTarget().isEmpty()) return;
        swerveSubsystem.applyRequest(() ->
                drive
                        .withVelocityX(-robotContainer.getDriverController().getLeftY()
                                * maxSpeed.magnitude())
                        .withTargetTx(Limelight.getTarget().get().position().getX())
                        .withRotationalRate(-robotContainer.getDriverController().getRightX()
                                * maxAngularRate.magnitude())
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) return;
        robotContainer.getGlobalStateMachine().transfer(Actions.INTERRUPT_SHOOT);
    }

    @Override
    public boolean isFinished() {
        return robotContainer.getGlobalStateMachine().getCurrentState() == StateMachine.States.PENDING;
    }
}
