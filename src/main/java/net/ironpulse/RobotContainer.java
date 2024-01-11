package net.ironpulse;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import net.ironpulse.Constants.OperatorConstants;
import net.ironpulse.subsystems.SwerveSubsystem;
import net.ironpulse.telemetries.SwerveTelemetry;

import static net.ironpulse.Constants.TunerConstants.maxAngularRate;
import static net.ironpulse.Constants.TunerConstants.maxSpeed;

public class RobotContainer {
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    public final SwerveSubsystem swerveSubsystem = Constants.TunerConstants.DriveTrain;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed.magnitude() * 0.1).withRotationalDeadband(maxAngularRate.magnitude() * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Command runAuto = swerveSubsystem.getAutoPath("Tests");

    private final SwerveTelemetry logger = new SwerveTelemetry(maxSpeed);

    private void configureBindings() {
        swerveSubsystem.setDefaultCommand(
                swerveSubsystem.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * maxSpeed.magnitude())
                        .withVelocityY(-driverController.getLeftX() * maxSpeed.magnitude())
                        .withRotationalRate(-driverController.getRightX() * maxAngularRate.magnitude())
                ).ignoringDisable(true));

        driverController.a().whileTrue(swerveSubsystem.applyRequest(() -> brake));
        driverController.b().whileTrue(swerveSubsystem
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        driverController.leftBumper().onTrue(swerveSubsystem.runOnce(swerveSubsystem::seedFieldRelative));
        swerveSubsystem.registerTelemetry(logger::telemeterize);

        driverController.pov(0).whileTrue(swerveSubsystem.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        driverController.pov(180).whileTrue(swerveSubsystem.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    }

    public Command getAutonomousCommand() {
        return runAuto;
    }
    

    public RobotContainer() {
        configureBindings();
    }
}
