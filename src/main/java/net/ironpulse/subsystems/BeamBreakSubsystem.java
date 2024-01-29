package net.ironpulse.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.Getter;
import net.ironpulse.RobotContainer;
import net.ironpulse.data.BeamBreakData;
import net.ironpulse.drivers.BeamBreak;
import net.ironpulse.state.StateMachine;

import java.util.function.Consumer;

import static net.ironpulse.Constants.BeamBreakConstants.*;

@Getter
public class BeamBreakSubsystem implements Subsystem {
    private final BeamBreak intakerBeamBreak;

    private final BeamBreak indexerBeamBreak;

    private final BeamBreak shooterLeftBeamBreak;

    private final RobotContainer robotContainer;
    private final Consumer<BeamBreakData> telemetryFunction;

    private final Timer timer = new Timer();

    public BeamBreakSubsystem(RobotContainer robotContainer, Consumer<BeamBreakData> telemetryFunction) {
        CommandScheduler.getInstance().registerSubsystem(this);
        this.telemetryFunction = telemetryFunction;
        this.robotContainer = robotContainer;
        intakerBeamBreak = new BeamBreak(INTAKER_BEAM_BREAK_ID);
        indexerBeamBreak = new BeamBreak(INDEXER_BEAM_BREAK_ID);
        shooterLeftBeamBreak = new BeamBreak(SHOOTER_BEAM_BREAK_ID);
    }

    @Override
    public void periodic() {
        telemetryFunction.accept(
                new BeamBreakData(
                        intakerBeamBreak.get(),
                        indexerBeamBreak.get(),
                        shooterLeftBeamBreak.get()
                )
        );

        if (!intakerBeamBreak.get() &&
                indexerBeamBreak.get() &&
                !shooterLeftBeamBreak.get()
        ) {
            robotContainer.getGlobalStateMachine()
                    .transfer(StateMachine.Actions.FINISH_INTAKE);
//            robotContainer.getDriverController().getHID()
//                    .setRumble(GenericHID.RumbleType.kBothRumble, 1);
            return;
        }

        if (!intakerBeamBreak.get() &&
                !indexerBeamBreak.get() &&
                !shooterLeftBeamBreak.get()
        ) {
            timer.start();
            if (!timer.hasElapsed(1)) return;
            robotContainer.getGlobalStateMachine()
                    .transfer(StateMachine.Actions.FINISH_SHOOT);
//            robotContainer.getDriverController().getHID()
//                    .setRumble(GenericHID.RumbleType.kBothRumble, 1);
            timer.stop();
            timer.reset();
        }
    }
}
