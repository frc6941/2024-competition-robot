package net.ironpulse;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import net.ironpulse.Constants.OperatorConstants;
import net.ironpulse.commands.*;
import net.ironpulse.commands.autos.AutoIntakeCommand;
import net.ironpulse.commands.autos.AutoPreShootCommand;
import net.ironpulse.commands.autos.AutoShootCommand;
import net.ironpulse.commands.manuals.*;
import net.ironpulse.subsystems.*;
import net.ironpulse.subsystems.drive.*;
import net.ironpulse.swerve.FieldCentricHeadingCorrect;
import net.ironpulse.telemetries.BeamBreakTelemetry;
import net.ironpulse.telemetries.IndexerTelemetry;
import net.ironpulse.telemetries.IntakerTelemetry;
import net.ironpulse.telemetries.ShooterTelemetry;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static net.ironpulse.Constants.SwerveConstants.maxAngularRate;
import static net.ironpulse.Constants.SwerveConstants.maxSpeed;

public class RobotContainer {
    @Getter
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final CommandXboxController operatorController =
            new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    private final IndexerTelemetry indexerTelemetry = new IndexerTelemetry();
    //    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(maxSpeed);
    private final IntakerTelemetry intakerTelemetry = new IntakerTelemetry();
    private final ShooterTelemetry shooterTelemetry = new ShooterTelemetry();
    private final BeamBreakTelemetry beamBreakTelemetry = new BeamBreakTelemetry();

    //    public final SwerveSubsystem swerveSubsystem = Constants.SwerveConstants.DriveTrain;
    private final Drive swerveSubsystem;
    //    private final Flywheel flywheel;
    public final IndexerSubsystem indexerSubsystem =
            new IndexerSubsystem(indexerTelemetry::telemeterize);

    @Getter
    public final BeamBreakSubsystem beamBreakSubsystem =
            new BeamBreakSubsystem(this, beamBreakTelemetry::telemeterize);
    private final IntakerSubsystem intakerSubsystem =
            new IntakerSubsystem(intakerTelemetry::telemeterize);
    @Getter
    private final ShooterSubsystem shooterSubsystem =
            new ShooterSubsystem(shooterTelemetry::telemeterize);

    @Getter
    private final IndicatorSubsystem indicatorSubsystem = new IndicatorSubsystem();

    private final FieldCentricHeadingCorrect drive = new FieldCentricHeadingCorrect()
            .withDeadband(maxSpeed.magnitude() * 0.1)
            .withRotationalDeadband(maxAngularRate.magnitude() * 0.1)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final LoggedDashboardChooser<Command> autoChooser;

    private Command autoCommand = null;

    private void configureKeyBindings() {
        swerveSubsystem.setDefaultCommand(
                DriveCommands.joystickDrive(
                        swerveSubsystem,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));

        driverController
                .b()
                .onTrue(Commands.runOnce(
                                () ->
                                        swerveSubsystem.setPose(
                                                new Pose2d(swerveSubsystem.getPose().getTranslation(), new Rotation2d())),
                                swerveSubsystem)
                        .ignoringDisable(true));
//        driverController.start().onTrue(swerveSubsystem.runOnce(swerveSubsystem::seedFieldRelative));
//        swerveSubsystem.registerTelemetry(swerveTelemetry::telemeterize);

        operatorController.rightTrigger().whileTrue(new SpeakerShootCommand(this, swerveSubsystem,
                shooterSubsystem, indexerSubsystem, () -> operatorController.getHID().getAButton()));
        operatorController.leftTrigger().whileTrue(new AmpShootCommand(this,
                shooterSubsystem, indexerSubsystem, () -> operatorController.getHID().getAButton()));

        driverController.rightBumper().whileTrue(
                Commands.parallel(
                        new IntakeCommand(this, intakerSubsystem),
                        new IndexCommand(this, indexerSubsystem)
                )
        );

        operatorController.pov(180).whileTrue(new ManualShooterUpCommand(shooterSubsystem));
        operatorController.pov(0).whileTrue(new ManualShooterDownCommand(shooterSubsystem))
                .and(() -> Rotations.of(shooterSubsystem.getArmMotor().getPosition().getValue()).in(Degrees) > 15);
        driverController.leftTrigger().whileTrue(Commands.parallel(
                new ManualIntakeInCommand(intakerSubsystem),
                new ManualIndexInCommand(indexerSubsystem)));
        driverController.rightTrigger().whileTrue(Commands.parallel(
                new ManualIntakeOutCommand(intakerSubsystem),
                new ManualIndexOutCommand(indexerSubsystem)));

        operatorController.start().onTrue(new ResetArmCommand(shooterSubsystem));
    }

    private void configureAutos() {
        NamedCommands.registerCommand("ShooterOn",
                new AutoPreShootCommand(shooterSubsystem));
        NamedCommands.registerCommand("AutoShoot",
                new AutoShootCommand(shooterSubsystem, indexerSubsystem));
        NamedCommands.registerCommand("Intake",
                new AutoIntakeCommand(intakerSubsystem, indexerSubsystem, beamBreakSubsystem));
    }

    public Command getAutonomousCommand() {
        if (autoCommand != null
                && autoCommand.getName().equals(autoChooser.get().getName() + " Decorator"))
            return autoCommand;
        var selected = autoChooser.get();
        autoCommand = selected.beforeStarting(
                Commands.runOnce(() -> resetOdometryWithAutoName(selected.getName())));
        autoCommand.setName(selected.getName() + " Decorator");
        return autoCommand;
    }

    private void resetOdometryWithAutoName(String autoName) {
        var pathGroup = PathPlannerAuto
                .getPathGroupFromAutoFile(autoName);
        if (pathGroup.isEmpty()) {
            // no path in auto
            return;
        }
        var startPose = pathGroup
                .get(0)
                .getPreviewStartingHolonomicPose();
        swerveSubsystem.setPose(flip() ?
                GeometryUtil.flipFieldPose(startPose) :
                startPose);
    }

    public static boolean flip() {
        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
    }

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                swerveSubsystem = new Drive(
                        new GyroIOPigeon2(true),
                        new ModuleIOTalonFX(0),
                        new ModuleIOTalonFX(1),
                        new ModuleIOTalonFX(2),
                        new ModuleIOTalonFX(3));
//                flywheel = new Flywheel(new FlywheelIOTalonFX());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                swerveSubsystem =
                        new Drive(
                                new GyroIO() {
                                },
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim());
//                flywheel = new Flywheel(new FlywheelIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                swerveSubsystem =
                        new Drive(
                                new GyroIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                });
//                flywheel = new Flywheel(new FlywheelIO() {});
                break;
        }

        configureAutos();
        configureKeyBindings();
        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser("M 1 Note Auto"));

        // Set up SysId routines
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
//        autoChooser.addOption(
//                "Flywheel SysId (Quasistatic Forward)",
//                flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//        autoChooser.addOption(
//                "Flywheel SysId (Quasistatic Reverse)",
//                flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//        autoChooser.addOption(
//                "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
//        autoChooser.addOption(
//                "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        indicatorSubsystem.setPattern(IndicatorSubsystem.Patterns.NORMAL);
    }
}
