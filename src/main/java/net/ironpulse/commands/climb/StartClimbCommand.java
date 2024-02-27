package net.ironpulse.commands.climb;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import net.ironpulse.commands.RumbleCommand;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;
import net.ironpulse.subsystems.swerve.SwerveSubsystem;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Seconds;

public class StartClimbCommand extends ParallelCommandGroup {
    public StartClimbCommand(
            ShooterSubsystem shooterSubsystem,
            SwerveSubsystem swerveSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            CommandXboxController driverController,
            CommandXboxController operatorController,
            BooleanSupplier confirmation
    ) {
        addCommands(
                new ClimbShooterUpCommand(shooterSubsystem),
                new ClimbDriveForwardCommand(swerveSubsystem),
                new SequentialCommandGroup(
                        Commands.runOnce(() -> indicatorSubsystem.setPattern(IndicatorIO.Patterns.CAN_CLIMB)),
                        new RumbleCommand(Seconds.of(0.5), driverController.getHID(), operatorController.getHID()),
                        new WaitUntilCommand(confirmation),
                        new ClimbEndgameCommand(shooterSubsystem, indicatorSubsystem)
                )
        );
    }
}
