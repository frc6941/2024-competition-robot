package net.ironpulse;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.Getter;
import net.ironpulse.Constants.OperatorConstants;
import net.ironpulse.commands.*;
import net.ironpulse.commands.autos.AutoIntakeCommand;
import net.ironpulse.commands.autos.AutoShootCommand;
import net.ironpulse.commands.manuals.ManualCleanStateWhileIntake;
import net.ironpulse.commands.manuals.ManualIndexInCommand;
import net.ironpulse.commands.manuals.ManualIndexOutCommand;
import net.ironpulse.commands.manuals.ManualIntakeInCommand;
import net.ironpulse.commands.manuals.ManualIntakeOutCommand;
import net.ironpulse.commands.manuals.ManualShootCommand;
import net.ironpulse.commands.manuals.ManualShooterDownCommand;
import net.ironpulse.commands.manuals.ManualShooterUpCommand;
import net.ironpulse.state.StateMachine;
import net.ironpulse.state.StateMachine.Actions;
import net.ironpulse.state.StateMachine.States;
import net.ironpulse.state.Transition;
import net.ironpulse.subsystems.*;
import net.ironpulse.telemetries.*;

import java.util.List;

import static net.ironpulse.Constants.SwerveConstants.maxAngularRate;
import static net.ironpulse.Constants.SwerveConstants.maxSpeed;

public class RobotContainer {
    @Getter
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final CommandXboxController operatorController =
            new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    private final IndexerTelemetry indexerTelemetry = new IndexerTelemetry();
    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(maxSpeed);
    private final IntakerTelemetry intakerTelemetry = new IntakerTelemetry();
    private final ShooterTelemetry shooterTelemetry = new ShooterTelemetry();
    private final BeamBreakTelemetry beamBreakTelemetry = new BeamBreakTelemetry();
    private final StateTelemetry stateTelemetry = new StateTelemetry();

    public final SwerveSubsystem swerveSubsystem = Constants.SwerveConstants.DriveTrain;
    public final IndexerSubsystem indexerSubsystem =
            new IndexerSubsystem(indexerTelemetry::telemeterize);
    public final BeamBreakSubsystem beamBreakSubsystem =
            new BeamBreakSubsystem(this, beamBreakTelemetry::telemeterize);
    private final IntakerSubsystem intakerSubsystem =
            new IntakerSubsystem(intakerTelemetry::telemeterize);
    private final ShooterSubsystem shooterSubsystem =
            new ShooterSubsystem(shooterTelemetry::telemeterize);
    private final StateSubsystem stateSubsystem =
            new StateSubsystem(this, stateTelemetry::telemeterize);
    private final IndicatorSubsystem indicatorSubsystem =
            new IndicatorSubsystem(this);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed.magnitude() * 0.1)
            .withRotationalDeadband(maxAngularRate.magnitude() * 0.1)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SendableChooser<Command> autoChooser =
            AutoBuilder.buildAutoChooser("M 1 Note Auto");

    private final List<Transition> transitions = List.of(
            Transition.builder()
                    .currentState(States.IDLE)
                    .nextState(States.INTAKING)
                    .action(Actions.INTAKE)
                    .build(),
            Transition.builder()
                    .currentState(States.INTAKING)
                    .nextState(States.PENDING)
                    .action(Actions.FINISH_INTAKE)
                    .command(new FinishIntakeLEDCommand(indicatorSubsystem))
                    .build(),
            Transition.builder()
                    .currentState(States.INTAKING)
                    .nextState(States.IDLE)
                    .action(Actions.INTERRUPT_INTAKE)
                    .command(new ResetIntakerCommand(intakerSubsystem))
                    .build(),
            Transition.builder()
                    .currentState(States.PENDING)
                    .nextState(States.SHOOTING)
                    .action(Actions.SHOOT)
                    .build(),
            Transition.builder()
                    .currentState(States.SHOOTING)
                    .nextState(States.PENDING)
                    .action(Actions.INTERRUPT_SHOOT)
                    .command(new ResetShooterCommand(shooterSubsystem))
                    .build(),
            Transition.builder()
                    .currentState(States.SHOOTING)
                    .nextState(States.IDLE)
                    .action(Actions.FINISH_SHOOT)
                    .command(Commands.sequence(
                            new ResetShooterCommand(shooterSubsystem),
                            new FinishShootLEDCommand(indicatorSubsystem)
                    ))
                    .build()
    );

    @Getter
    private final StateMachine globalStateMachine = new StateMachine(States.IDLE, transitions);
    
    private void configureKeyBindings() {
        swerveSubsystem.setDefaultCommand(swerveSubsystem
                .applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * maxSpeed.magnitude())
                        .withVelocityY(-driverController.getLeftX() * maxSpeed.magnitude())
                        .withRotationalRate(-driverController.getRightX() * maxAngularRate.magnitude()))
                .ignoringDisable(true));

        driverController
                .a()
                .whileTrue(swerveSubsystem.applyRequest(() -> brake));
        driverController
                .b()
                .whileTrue(swerveSubsystem.applyRequest(() -> point.withModuleDirection(
                        new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        driverController.leftBumper().onTrue(swerveSubsystem.runOnce(swerveSubsystem::seedFieldRelative));
        swerveSubsystem.registerTelemetry(swerveTelemetry::telemeterize);

       


        driverController.rightTrigger().whileTrue(new SpeakerShootCommand(this, swerveSubsystem,
                shooterSubsystem, indexerSubsystem, () -> driverController.getHID().getAButton()));
        driverController.leftTrigger().whileTrue(new AmpShootCommand(this,
                shooterSubsystem, indexerSubsystem, swerveSubsystem, () -> driverController.getHID().getAButton()));

        driverController.rightBumper().whileTrue(
                Commands.parallel(
                        new IntakeCommand(this, intakerSubsystem),
                        new IndexCommand(this, indexerSubsystem)
                )
        );

        // TODO Bind operator manual actions
        driverController.pov(0).whileTrue(new ManualShooterUpCommand(shooterSubsystem));
        driverController.pov(180).whileTrue(new ManualShooterDownCommand(shooterSubsystem));
        driverController.pov(90).whileTrue(Commands.parallel(
                new ManualIntakeInCommand(intakerSubsystem),
                new ManualIndexInCommand(indexerSubsystem)));
        driverController.pov(270).whileTrue(Commands.parallel(
                new ManualIntakeOutCommand(intakerSubsystem),
                new ManualIndexOutCommand(indexerSubsystem)));

        driverController.a().whileTrue(new ManualShootCommand(shooterSubsystem));
        driverController.x().whileTrue(new ManualCleanStateWhileIntake(this));
    }

    private void configureAutos() {
        NamedCommands.registerCommand("AutoShoot",
                new AutoShootCommand(shooterSubsystem, indexerSubsystem));
        NamedCommands.registerCommand("Intake",
                new AutoIntakeCommand(intakerSubsystem, indexerSubsystem, beamBreakSubsystem));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
    public RobotContainer() {
        configureAutos();
        configureKeyBindings();
    }
}
