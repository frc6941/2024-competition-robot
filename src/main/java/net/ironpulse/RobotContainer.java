package net.ironpulse;

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
import net.ironpulse.commands.*;
import net.ironpulse.commands.autos.AutoIntakeCommand;
import net.ironpulse.commands.autos.AutoPreShootCommand;
import net.ironpulse.commands.autos.AutoShootCommand;
import net.ironpulse.subsystems.beambreak.BeamBreakIORev;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indexer.IndexerIOTalonFX;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.intaker.IntakerIOTalonFX;
import net.ironpulse.subsystems.intaker.IntakerSubsystem;
import net.ironpulse.subsystems.shooter.ShooterIOTalonFX;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;
import net.ironpulse.subsystems.swerve.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static edu.wpi.first.units.Units.Seconds;

public class RobotContainer {
    private final CommandXboxController driverController =
            new CommandXboxController(0);

    private final CommandXboxController operatorController =
            new CommandXboxController(1);

    private SwerveSubsystem swerveSubsystem;
    private IntakerSubsystem intakerSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private BeamBreakSubsystem beamBreakSubsystem;

    private LoggedDashboardChooser<Command> autoChooser;
    private Command autoCommand = null;

    private void configureKeyBindings() {
        swerveSubsystem.setDefaultCommand(
                new DefaultDriveCommand(
                        swerveSubsystem,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));
        driverController.x().onTrue(Commands.runOnce(swerveSubsystem::stopWithX, swerveSubsystem));
        driverController.b().onTrue(
                Commands.runOnce(() -> swerveSubsystem.setPose(
                        new Pose2d(swerveSubsystem.getPose().getTranslation(), new Rotation2d())),
                                swerveSubsystem)
                        .ignoringDisable(true));

        driverController.rightBumper().whileTrue(Commands.runEnd(
                () -> Commands.parallel(
                        new IntakeCommand(intakerSubsystem, beamBreakSubsystem),
                        new IndexCommand(indexerSubsystem, beamBreakSubsystem)
                ),
                () -> new RumbleCommand(driverController.getHID(), Seconds.of(1))
        ));

        operatorController.rightTrigger().whileTrue(
                new SpeakerShootCommand(
                        swerveSubsystem,
                        shooterSubsystem,
                        indexerSubsystem,
                        beamBreakSubsystem,
                        () -> operatorController.getHID().getAButton(),
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX()
                )
        );

        operatorController.rightTrigger().whileTrue(
                new AmpShootCommand(
                        shooterSubsystem,
                        indexerSubsystem,
                        beamBreakSubsystem,
                        () -> operatorController.getHID().getAButton()
                )
        );

        operatorController.x().whileTrue(
                new ParallelShootCommand(
                        shooterSubsystem,
                        indexerSubsystem,
                        beamBreakSubsystem,
                        () -> operatorController.getHID().getAButton()
                )
        );

        operatorController.start().onTrue(new ResetArmCommand(shooterSubsystem));
    }

    private void configureAutos() {
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        NamedCommands.registerCommand("ShooterOn",
                new AutoPreShootCommand(shooterSubsystem));
        NamedCommands.registerCommand("AutoShoot",
                new AutoShootCommand(shooterSubsystem, indexerSubsystem));
        NamedCommands.registerCommand("Intake",
                new AutoIntakeCommand(intakerSubsystem, indexerSubsystem, beamBreakSubsystem));
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
        swerveSubsystem.setPose(flip() ?
                GeometryUtil.flipFieldPose(startPose) :
                startPose);
    }

    public static boolean flip() {
        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
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
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                swerveSubsystem =
                        new SwerveSubsystem(
                                new GyroIO() {},
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim());
                break;
        }
    }

    public RobotContainer() {
        configureSubsystem();
        configureAutos();
        configureKeyBindings();
    }
}
