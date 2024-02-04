package net.ironpulse;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.Getter;
import net.ironpulse.Constants.OperatorConstants;
import net.ironpulse.commands.*;
import net.ironpulse.commands.autos.AutoIntakeCommand;
import net.ironpulse.commands.autos.AutoPreShootCommand;
import net.ironpulse.commands.autos.AutoShootCommand;
import net.ironpulse.commands.autos.AutoShootWithAngleCommand;
import net.ironpulse.commands.manuals.*;
import net.ironpulse.maths.MathMisc;
import net.ironpulse.subsystems.*;
import net.ironpulse.telemetries.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static net.ironpulse.Constants.SwerveConstants.*;

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

    public final SwerveSubsystem swerveSubsystem = Constants.SwerveConstants.DriveTrain;
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

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed.magnitude() * 0.1)
            .withRotationalDeadband(maxAngularRate.magnitude() * 0.1)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final LoggedDashboardChooser<Command> autoChooser;

    private Command autoCommand = null;

    private void configureKeyBindings() {
        swerveSubsystem.setDefaultCommand(swerveSubsystem
                .applyRequest(() -> drive.withVelocityX(MathMisc.sign(-driverController.getLeftY())
                                * xLimiter.calculate(Math.abs(driverController.getLeftY())) * maxSpeed.magnitude())
                        .withVelocityY(MathMisc.sign(-driverController.getLeftX()) * yLimiter.calculate(Math.abs(driverController.getLeftX())) * maxSpeed.magnitude())
                        .withRotationalRate(-driverController.getRightX() * maxAngularRate.magnitude()))
                .ignoringDisable(true));

        driverController.b().whileTrue(swerveSubsystem.applyRequest(() -> brake));

        driverController.start().onTrue(swerveSubsystem.runOnce(swerveSubsystem::seedFieldRelative));
        swerveSubsystem.registerTelemetry(swerveTelemetry::telemeterize);

        operatorController.rightTrigger().whileTrue(new SpeakerShootCommand(this, swerveSubsystem,
                shooterSubsystem, indexerSubsystem, () -> operatorController.getHID().getAButton()));
        operatorController.leftTrigger().whileTrue(new AmpShootCommand(this,
                shooterSubsystem, indexerSubsystem, () -> operatorController.getHID().getAButton()));

        operatorController.a().whileTrue(new ShootWithoutAimingCommand(this, shooterSubsystem, 
                indexerSubsystem, () -> operatorController.getHID().getRightBumper()));
        operatorController.b().whileTrue(new ParallelShootCommand(this, shooterSubsystem,           
                indexerSubsystem, () -> operatorController.getHID().getRightBumper(), 30));
        operatorController.x().whileTrue(new ParallelShootCommand(this, shooterSubsystem,
                indexerSubsystem, () -> operatorController.getHID().getRightBumper(), 46));
        operatorController.y().whileTrue(new ParallelShootCommand(this, shooterSubsystem,
                indexerSubsystem, () -> operatorController.getHID().getRightBumper(), 62));




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
        NamedCommands.registerCommand("AutoPreShoot",
                new AutoPreShootCommand(shooterSubsystem));
        NamedCommands.registerCommand("Intake",
                new AutoIntakeCommand(intakerSubsystem, indexerSubsystem, beamBreakSubsystem));
        NamedCommands.registerCommand("ShootNearSpeaker",
                new AutoShootWithAngleCommand(shooterSubsystem, indexerSubsystem, 30));
        NamedCommands.registerCommand("ShootOnLine", 
                new AutoShootWithAngleCommand(shooterSubsystem, indexerSubsystem, 46));
        NamedCommands.registerCommand("ShootAtLaunchPad", 
                new AutoShootWithAngleCommand(shooterSubsystem, indexerSubsystem, 62));
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
        swerveSubsystem.seedFieldRelative(flip() ?
                GeometryUtil.flipFieldPose(startPose) :
                startPose);
    }

    public static boolean flip() {
        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
    }

    public RobotContainer() {
        configureAutos();
        configureKeyBindings();
        indicatorSubsystem.setPattern(IndicatorSubsystem.Patterns.NORMAL);
        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser("M 1 Note Auto"));
    }
}
