package net.ironpulse;

import com.ctre.phoenix6.controls.Follower;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import net.ironpulse.utils.LocalADStarAK;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    public void configureLogger() {
//        Logger.recordMetadata("ProjectName", "2024-competition-robot");
//        Logger.addDataReceiver(new NT4Publisher());
//        if (!isReal()) {
//            // Run as fast as possible
//            setUseTiming(false);
//        }

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL, SIM:
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }
        // Detailed swerve logging not implemented;
        // see https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/COMMON-ISSUES.md#non-deterministic-data-sources
        Logger.start();
    }

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStarAK());
        configureLogger();
        robotContainer = new RobotContainer();
//        robotContainer.swerveSubsystem.getDaqThread().setThreadPriority(99);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand == null) return;
        autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        robotContainer.getShooterSubsystem().getShootMotorLeft()
                .setVoltage(Constants.ShooterConstants.shooterConstantVoltage.magnitude());
        robotContainer.getShooterSubsystem().getShootMotorRight()
                .setControl(
                        new Follower(Constants.ShooterConstants.SHOOTER_L_MOTOR_ID,
                                true)
                );
        if (autonomousCommand == null) return;
        autonomousCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
