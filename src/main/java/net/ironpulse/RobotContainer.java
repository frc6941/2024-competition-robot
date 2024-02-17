package net.ironpulse;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import net.ironpulse.commands.*;
import net.ironpulse.commands.autos.AutoIntakeCommand;
import net.ironpulse.commands.autos.AutoPreShootCommand;
import net.ironpulse.commands.autos.AutoShootCommand;
import net.ironpulse.commands.autos.AutoShootWithAngleCommand;
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
import net.ironpulse.subsystems.swerve.*;
import net.ironpulse.utils.Utils;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static edu.wpi.first.units.Units.*;
import static net.ironpulse.Constants.ShooterConstants.shooterConstantVoltage;

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
    private Command autoCommand = null;

    private void configureKeyBindings() {
        swerveSubsystem.setDefaultCommand(
                new DefaultDriveCommand(
                        swerveSubsystem,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));
        driverController.b().onTrue(Commands.runOnce(swerveSubsystem::stopWithX, swerveSubsystem));
        driverController.start().onTrue(
                Commands.runOnce(() -> swerveSubsystem.setPose(
                                        new Pose2d(swerveSubsystem.getPose().getTranslation(), new Rotation2d())),
                                swerveSubsystem)
                        .ignoringDisable(true));

        driverController.rightBumper().whileTrue(
                Commands.sequence(
                        Commands.parallel(
                                new IntakeCommand(intakerSubsystem, beamBreakSubsystem, indicatorSubsystem),
                                new IndexCommand(indexerSubsystem, beamBreakSubsystem)
                        ),
                        new RumbleCommand(driverController.getHID(), Seconds.of(1))
                )
        );
        driverController.leftTrigger().whileTrue(
                Commands.sequence(
                        Commands.parallel(
                                new IntakeCommand(intakerSubsystem, beamBreakSubsystem, indicatorSubsystem),
                                new IndexCommand(indexerSubsystem, beamBreakSubsystem)
                        ),
                        new RumbleCommand(driverController.getHID(), Seconds.of(1))
                )
        );
        driverController.rightTrigger().whileTrue(Commands.parallel(
                new IntakeOutCommand(intakerSubsystem),
                new IndexOutCommand(indexerSubsystem)));


        operatorController.rightTrigger().whileTrue(
                Commands.sequence(
                        new SpeakerShootCommand(
                                swerveSubsystem,
                                shooterSubsystem,
                                indexerSubsystem,
                                beamBreakSubsystem,
                                indicatorSubsystem,
                                () -> operatorController.getHID().getAButton(),
                                () -> -driverController.getLeftY(),
                                () -> -driverController.getLeftX()
                        ),
                        new RumbleCommand(driverController.getHID(), Seconds.of(1))
                )
        );

        operatorController.leftTrigger().whileTrue(
                Commands.sequence(
                        new AmpShootCommand(
                                shooterSubsystem,
                                indexerSubsystem,
                                beamBreakSubsystem,
                                indicatorSubsystem,
                                () -> operatorController.getHID().getAButton()
                        ),
                        new RumbleCommand(driverController.getHID(), Seconds.of(1))
                )
        );

        operatorController.a().whileTrue(new ShootWithoutAimingCommand(indicatorSubsystem, beamBreakSubsystem,
                shooterSubsystem, indexerSubsystem, () -> operatorController.getHID().getRightBumper()));
        operatorController.b().whileTrue(new ParallelShootCommand(shooterSubsystem,
                indexerSubsystem, beamBreakSubsystem, indicatorSubsystem,
                () -> operatorController.getHID().getRightBumper(), Degrees.of(30)));
        operatorController.x().whileTrue(new ParallelShootCommand(shooterSubsystem,
                indexerSubsystem, beamBreakSubsystem, indicatorSubsystem,
                () -> operatorController.getHID().getRightBumper(), Degrees.of(46)));
        operatorController.y().whileTrue(new ParallelShootCommand(shooterSubsystem,
                indexerSubsystem, beamBreakSubsystem, indicatorSubsystem,
                () -> operatorController.getHID().getRightBumper(), Degrees.of(62)));

        operatorController.pov(180).whileTrue(new ShooterUpCommand(shooterSubsystem));
        operatorController.pov(0).whileTrue(new ShooterDownCommand(shooterSubsystem))
                .and(() -> Rotations.of(shooterSubsystem.getInputs().armPosition.magnitude()).in(Degrees) > 15);

        operatorController.back().toggleOnTrue(new RainbowCommand(indicatorSubsystem));

        operatorController.start().onTrue(new ResetArmCommand(shooterSubsystem));
    }

    private void configureAutos() {
        NamedCommands.registerCommand("ShooterOn",
                new AutoPreShootCommand(shooterSubsystem));
        NamedCommands.registerCommand("AutoShoot",
                new AutoShootCommand(shooterSubsystem, indexerSubsystem));
        NamedCommands.registerCommand("Intake",
                new AutoIntakeCommand(intakerSubsystem, indexerSubsystem, beamBreakSubsystem));
        NamedCommands.registerCommand("AutoPreShoot",
                new AutoPreShootCommand(shooterSubsystem));
        NamedCommands.registerCommand("ShootNearSpeaker",
                new AutoShootWithAngleCommand(shooterSubsystem, indexerSubsystem, 26));
        NamedCommands.registerCommand("ShootOnLine",
                new AutoShootWithAngleCommand(shooterSubsystem, indexerSubsystem, 46));
        NamedCommands.registerCommand("ShootAtLaunchPad",
                new AutoShootWithAngleCommand(shooterSubsystem, indexerSubsystem, 62));
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser("Choreo Half"));
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
        swerveSubsystem.setPose(Utils.flip() ?
                GeometryUtil.flipFieldPose(startPose) :
                startPose);
    }

    private void configureSubsystem() {
        switch (Constants.currentMode) {
            default:
            case REAL:
                swerveSubsystem = new SwerveSubsystem(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(0),
                        new ModuleIOTalonFX(1),
                        new ModuleIOTalonFX(2),
                        new ModuleIOTalonFX(3)
                );
                intakerSubsystem = new IntakerSubsystem(new IntakerIOTalonFX());
                indexerSubsystem = new IndexerSubsystem(new IndexerIOTalonFX());
                shooterSubsystem = new ShooterSubsystem(new ShooterIOTalonFX());
                beamBreakSubsystem = new BeamBreakSubsystem(new BeamBreakIORev());
                indicatorSubsystem = new IndicatorSubsystem(new IndicatorIOARGB());
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                swerveSubsystem =
                        new SwerveSubsystem(
                                inputs -> {},
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim());
                indicatorSubsystem = new IndicatorSubsystem(new IndicatorIOSim());
                intakerSubsystem = new IntakerSubsystem(new IntakerIOSim());
                shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
                beamBreakSubsystem = new BeamBreakSubsystem(new BeamBreakIOSim());
                indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());
                break;
        }
    }

    public RobotContainer() {
        configureSubsystem();
        configureAutos();
        configureKeyBindings();
        indicatorSubsystem.setPattern(IndicatorIO.Patterns.NORMAL);
        shooterSubsystem.getIo().setShooterVoltage(shooterConstantVoltage);
    }
}