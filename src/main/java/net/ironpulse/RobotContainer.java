package net.ironpulse;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.Getter;
import net.ironpulse.commands.*;
import net.ironpulse.commands.autos.*;
import net.ironpulse.commands.climb.ClimbCommand;
import net.ironpulse.commands.climb.ClimbManualShooterUpCommand;
import net.ironpulse.commands.climb.StartClimbCommand;
import net.ironpulse.subsystems.beambreak.BeamBreakIORev;
import net.ironpulse.subsystems.beambreak.BeamBreakIOSim;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indexer.IndexerIOSim;
import net.ironpulse.subsystems.indexer.IndexerIOTalonFX;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.indicator.IndicatorIOARGB;
import net.ironpulse.subsystems.indicator.IndicatorIOSim;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.intaker.IntakerIOSim;
import net.ironpulse.subsystems.intaker.IntakerIOTalonFX;
import net.ironpulse.subsystems.intaker.IntakerSubsystem;
import net.ironpulse.subsystems.shooter.ShooterIOSim;
import net.ironpulse.subsystems.shooter.ShooterIOTalonFX;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;
import net.ironpulse.subsystems.swerve.SwerveSubsystem;
import net.ironpulse.utils.Utils;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import static net.ironpulse.Constants.SwerveConstants.*;
import static net.ironpulse.utils.Utils.*;

public class RobotContainer {
    @Getter
    private final CommandXboxController driverController =
            new CommandXboxController(0);

    @Getter
    private final CommandXboxController operatorController =
            new CommandXboxController(1);

    private SwerveSubsystem swerveSubsystem;
    private IntakerSubsystem intakerSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private BeamBreakSubsystem beamBreakSubsystem;
    private IndicatorSubsystem indicatorSubsystem;

    private LoggedDashboardChooser<Command> autoChooser;
    private final LoggedDashboardNumber shootAngle = new LoggedDashboardNumber("Shoot Angle", 0.0);
    private Command autoCommand = null;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed.magnitude() * 0.1)
            .withRotationalDeadband(maxAngularRate.magnitude() * 0.1)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private void configureKeyBindings() {
        swerveSubsystem.setDefaultCommand(swerveSubsystem
                .applyRequest(
                        () -> drive
                                .withVelocityX(Utils.sign(-driverController.getLeftY())
                                        * xLimiter.calculate(Math.abs(driverController.getLeftY()))
                                        * maxSpeed.magnitude())
                                .withVelocityY(
                                        Utils.sign(-driverController.getLeftX()) * maxSpeed.magnitude()
                                                * yLimiter.calculate(Math.abs(driverController.getLeftX()))
                                )
                                .withRotationalRate(
                                        -driverController.getRightX() * maxAngularRate.magnitude()
                                )
                )
                .ignoringDisable(true));

        driverController.b().whileTrue(swerveSubsystem.applyRequest(() -> brake));

        driverController.start().onTrue(swerveSubsystem.runOnce(swerveSubsystem::seedFieldRelative));

        driverController.rightBumper().whileTrue(
                Commands.sequence(
                        Commands.parallel(
                                new IntakeCommand(intakerSubsystem, beamBreakSubsystem, indicatorSubsystem),
                                new IndexCommand(indexerSubsystem, beamBreakSubsystem)
                        ),
                        new RumbleCommand(Seconds.of(1), driverController.getHID(), operatorController.getHID())
                )
        );
        driverController.leftTrigger().whileTrue(
                Commands.sequence(
                        Commands.parallel(
                                new IntakeCommand(intakerSubsystem, beamBreakSubsystem, indicatorSubsystem),
                                new IndexCommand(indexerSubsystem, beamBreakSubsystem)
                        ),
                        new RumbleCommand(Seconds.of(1), driverController.getHID(), operatorController.getHID())
                )
        );
        driverController.rightTrigger().whileTrue(Commands.parallel(
                new IntakeOutCommand(intakerSubsystem),
                new IndexOutCommand(indexerSubsystem)));

        driverController.leftBumper().whileTrue(new ParallelShootWithDelayCommand(shooterSubsystem,
                indexerSubsystem, beamBreakSubsystem, indicatorSubsystem, Degrees.of(20)));

        operatorController.rightTrigger().whileTrue(
                Commands.sequence(
                        new SpeakerShootCommand(
                                shooterSubsystem,
                                indexerSubsystem,
                                beamBreakSubsystem,
                                indicatorSubsystem,
                                swerveSubsystem,
                                driverController,
                                () -> operatorController.getHID().getRightBumper()
                        ),
                        new RumbleCommand(Seconds.of(1), driverController.getHID(), operatorController.getHID())
                )
        );

        operatorController.leftTrigger().whileTrue(
                Commands.sequence(
                        new AmpShootCommand(
                                shooterSubsystem,
                                indexerSubsystem,
                                beamBreakSubsystem,
                                indicatorSubsystem,
                                () -> operatorController.getHID().getRightBumper()
                        ),
                        new RumbleCommand(Seconds.of(1), driverController.getHID(), operatorController.getHID())
                )
        );

        operatorController.b().whileTrue(new ShootWithoutAimingCommand(indicatorSubsystem, beamBreakSubsystem,
                shooterSubsystem, indexerSubsystem, () -> operatorController.getHID().getRightBumper()));
        operatorController.x().whileTrue(new ParallelShootCommand(shooterSubsystem,
                indexerSubsystem, beamBreakSubsystem, indicatorSubsystem,
                () -> operatorController.getHID().getRightBumper(), Degrees.of(95)));
        operatorController.a().toggleOnTrue(
                new StartClimbCommand(shooterSubsystem, indicatorSubsystem,
                        driverController, operatorController,
                        () -> operatorController.getHID().getYButton())
        );

        operatorController.pov(180).whileTrue(new ShooterDownCommand(shooterSubsystem));
        operatorController.pov(0).whileTrue(new ClimbManualShooterUpCommand(shooterSubsystem));
        operatorController.pov(90).whileTrue(new ClimbCommand(shooterSubsystem, false));
        operatorController.pov(270).whileTrue(new ClimbCommand(shooterSubsystem, true));

//        driverController.pov(0).whileTrue(new ParallelShootCommand(shooterSubsystem,
//                indexerSubsystem, beamBreakSubsystem, indicatorSubsystem,
//                () -> driverController.getHID().getAButton(), shootAngle));

//        operatorController.pov(0).whileTrue(Commands.runOnce(() -> {
//            Constants.ShooterConstants.speakerArmOffset = Constants.ShooterConstants.speakerArmOffset.plus(Degrees.of(0.5));
//        }));
//        operatorController.pov(0).whileTrue(Commands.runOnce(() -> {
//            Constants.ShooterConstants.speakerArmOffsetNear = Constants.ShooterConstants.speakerArmOffsetNear.plus(Degrees.of(0.5));
//        }));
//        operatorController.pov(180).whileTrue(Commands.runOnce(() -> {
//            Constants.ShooterConstants.speakerArmOffset = Constants.ShooterConstants.speakerArmOffset.minus(Degrees.of(0.5));
//        }));
//        operatorController.pov(90).whileTrue(Commands.runOnce(() -> {
//            Constants.ShooterConstants.speakerArmOffsetNear = Constants.ShooterConstants.speakerArmOffsetNear.minus(Degrees.of(0.5));
//        }));
//        driverController.y().whileTrue(Commands.runOnce(() -> {
//            Constants.ShooterConstants.speakerArmOffsetFar = Constants.ShooterConstants.speakerArmOffsetFar.plus(Degrees.of(0.5));
//        }));
//        driverController.a().whileTrue(Commands.runOnce(() -> {
//            Constants.ShooterConstants.speakerArmOffsetFar = Constants.ShooterConstants.speakerArmOffsetFar.minus(Degrees.of(0.5));
//        }));
        // Remember if you will, or, better still, forget it.
//        operatorController.pov(0).toggleOnTrue(new ShootPlateCommand(
//                shooterSubsystem,
//                indexerSubsystem,
//                beamBreakSubsystem,
//                () -> operatorController.getHID().getRightBumper()
//        ));

        operatorController.back().toggleOnTrue(new RainbowCommand(indicatorSubsystem));

        operatorController.start().onTrue(new ResetArmCommand(shooterSubsystem));
    }

    private void configureAutos() {
        NamedCommands.registerCommand("AutoShoot",
                new AutoShootCommand(indexerSubsystem, beamBreakSubsystem));
        NamedCommands.registerCommand("ShootManual",
                new AutoDeliverNoteCommand(indexerSubsystem, beamBreakSubsystem));
        NamedCommands.registerCommand("Intake",
                new AutoIntakeCommand(intakerSubsystem, indexerSubsystem, beamBreakSubsystem));
        NamedCommands.registerCommand("IntakeFar",
                new AutoIntakeFarCommand(intakerSubsystem, indexerSubsystem, beamBreakSubsystem));
        NamedCommands.registerCommand("AutoPreShoot",
                new AutoPreShootCommand(shooterSubsystem));
        NamedCommands.registerCommand("Mess",
                new AutoMessCommand(intakerSubsystem, indexerSubsystem, shooterSubsystem));
        NamedCommands.registerCommand("ResetArm",
                new ResetArmCommand(shooterSubsystem));
        NamedCommands.registerCommand("StartBlind",
                Commands.runOnce(() -> blind = true));
        NamedCommands.registerCommand("EndBlind",
                Commands.runOnce(() -> blind = false));
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
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
        swerveSubsystem.seedFieldRelative(Utils.flip() ?
                GeometryUtil.flipFieldPose(startPose) :
                startPose);
    }

    private void configureSubsystem() {
        switch (Constants.currentMode) {
            default:
            case REAL:
                swerveSubsystem = DriveTrain;
                intakerSubsystem = new IntakerSubsystem(new IntakerIOTalonFX());
                indexerSubsystem = new IndexerSubsystem(new IndexerIOTalonFX());
                shooterSubsystem = new ShooterSubsystem(new ShooterIOTalonFX());
                beamBreakSubsystem = new BeamBreakSubsystem(new BeamBreakIORev());
                indicatorSubsystem = new IndicatorSubsystem(new IndicatorIOARGB());
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                swerveSubsystem = DriveTrain;
                indicatorSubsystem = new IndicatorSubsystem(new IndicatorIOSim());
                intakerSubsystem = new IntakerSubsystem(new IntakerIOSim());
                shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
                beamBreakSubsystem = new BeamBreakSubsystem(new BeamBreakIOSim());
                indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());
                break;
        }
    }

    public void configureStates() {
        autoIntaking = false;
        blind = false;
        armReachedClimb = false;
        indicatorSubsystem.setPattern(IndicatorIO.Patterns.NORMAL);
    }

    public RobotContainer() {
        configureSubsystem();
        configureAutos();
        configureKeyBindings();
        configureStates();
    }
}