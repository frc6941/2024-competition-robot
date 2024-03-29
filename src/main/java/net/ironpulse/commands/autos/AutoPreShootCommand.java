package net.ironpulse.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;

public class AutoPreShootCommand extends ParallelCommandGroup {
    public AutoPreShootCommand(
            ShooterSubsystem shooterSubsystem
    ) {
        addCommands(
                new AutoAimingCommand(shooterSubsystem),
                new AutoShooterSpeedUpCommand(shooterSubsystem)
        );
    }
}
