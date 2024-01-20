package net.ironpulse;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.Getter;
import net.ironpulse.Constants.OperatorConstants;
import net.ironpulse.commands.AimingCommand;
import net.ironpulse.commands.IndexCommand;
import net.ironpulse.commands.IntakeCommand;
import net.ironpulse.state.StateMachine;
import net.ironpulse.state.Transition;
import net.ironpulse.subsystems.BeamBreakSubsystem;
import net.ironpulse.subsystems.IndexerSubsystem;
import net.ironpulse.subsystems.IntakerSubsystem;
import net.ironpulse.subsystems.SwerveSubsystem;
import net.ironpulse.telemetries.IndexerTelemetry;
import net.ironpulse.telemetries.SwerveTelemetry;

import java.util.List;

import static net.ironpulse.Constants.SwerveConstants.maxAngularRate;
import static net.ironpulse.Constants.SwerveConstants.maxSpeed;

public class RobotContainer {
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final IndexerTelemetry indexerTelemetry = new IndexerTelemetry();
    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(maxSpeed);

    public final SwerveSubsystem swerveSubsystem = Constants.SwerveConstants.DriveTrain;
    public final IndexerSubsystem indexerSubsystem = new IndexerSubsystem(indexerTelemetry::telemeterize);
    public final BeamBreakSubsystem beamBreakSubsystem = new BeamBreakSubsystem(this);
    private final IntakerSubsystem intakerSubsystem = new IntakerSubsystem(a -> {});

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed.magnitude() * 0.1)
            .withRotationalDeadband(maxAngularRate.magnitude() * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight =
            new SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Command runAuto = swerveSubsystem.getAutoPath("Tests");

    public enum States {
        IDLE, INTAKING, PENDING, AIMING, SHOOTING
    }

    public enum Actions {
        INTAKE, FINISH_INTAKE, SHOOT, AIM, FINISH_SHOOT, INTERRUPT_INTAKE
    }

    private final List<Transition> transitions = List.of(
            Transition.builder()
                    .currentState(States.IDLE)
                    .nextState(States.INTAKING)
                    .action(Actions.INTAKE)
                    .command(new IndexCommand(this, indexerSubsystem))
                    .build(),
            Transition.builder()
                    .currentState(States.INTAKING)
                    .nextState(States.PENDING)
                    .action(Actions.FINISH_INTAKE)
                    .build(),
            Transition.builder()
                    .currentState(States.INTAKING)
                    .nextState(States.IDLE)
                    .action(Actions.INTERRUPT_INTAKE)
                    .build(),
            Transition.builder()
                    .currentState(States.PENDING)
                    .nextState(States.AIMING)
                    .action(Actions.AIM)
                    .build(),
            Transition.builder()
                    .currentState(States.AIMING)
                    .nextState(States.SHOOTING)
                    .action(Actions.SHOOT)
                    .build(),
            Transition.builder()
                    .currentState(States.SHOOTING)
                    .nextState(States.IDLE)
                    .action(Actions.FINISH_SHOOT)
                    .build()
    );

    @Getter
    private final StateMachine globalState = new StateMachine(States.IDLE, transitions);

    private void configureKeyBindings() {
        swerveSubsystem.setDefaultCommand(swerveSubsystem
                .applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * maxSpeed.magnitude())
                        .withVelocityY(-driverController.getLeftX() * maxSpeed.magnitude())
                        .withRotationalRate(-driverController.getRightX() * maxAngularRate.magnitude()))
                .ignoringDisable(true));

        driverController.a().whileTrue(swerveSubsystem.applyRequest(() -> brake));
        driverController
                .b()
                .whileTrue(swerveSubsystem.applyRequest(() -> point.withModuleDirection(
                        new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        driverController.leftBumper().onTrue(swerveSubsystem.runOnce(swerveSubsystem::seedFieldRelative));
        swerveSubsystem.registerTelemetry(swerveTelemetry::telemeterize);

        driverController.pov(0).whileTrue(swerveSubsystem.applyRequest(() -> forwardStraight
                .withVelocityX(0.5)
                .withVelocityY(0)));
        driverController.pov(180).whileTrue(swerveSubsystem.applyRequest(() -> forwardStraight
                .withVelocityX(-0.5)
                .withVelocityY(0)));

        driverController.x().whileTrue(new AimingCommand(swerveSubsystem));
        driverController.rightBumper().whileTrue(new IntakeCommand(this, intakerSubsystem));
    }

    public Command getAutonomousCommand() {
        return runAuto;
    }

    public RobotContainer() {
        configureKeyBindings();
    }
}
