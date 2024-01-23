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

    private final BeamBreak shooterBeamBreak;

    private final RobotContainer robotContainer;
    private final Consumer<BeamBreakData> telemetryFunction;

    private final Timer timer = new Timer();

    public BeamBreakSubsystem(RobotContainer robotContainer, Consumer<BeamBreakData> telemetryFunction) {
        CommandScheduler.getInstance().registerSubsystem(this);
        this.telemetryFunction = telemetryFunction;
        this.robotContainer = robotContainer;
        intakerBeamBreak = new BeamBreak(INTAKER_BEAM_BREAK_ID);
        indexerBeamBreak = new BeamBreak(INDEXER_BEAM_BREAK_ID);
        shooterBeamBreak = new BeamBreak(SHOOTER_BEAM_BREAK_ID);
    }

    @Override
    public void periodic() {
        telemetryFunction.accept(
                new BeamBreakData(
                        intakerBeamBreak.get(),
                        indexerBeamBreak.get(),
                        shooterBeamBreak.get()
                )
        );
        if (!intakerBeamBreak.get() && indexerBeamBreak.get() && !shooterBeamBreak.get()) {
            robotContainer.getGlobalStateMachine().transfer(StateMachine.Actions.FINISH_INTAKE);
            return;
        }

        if (!intakerBeamBreak.get() && !indexerBeamBreak.get() && !shooterBeamBreak.get()) {
            timer.start();
            if (!timer.hasElapsed(0.5)) return;
            robotContainer.getGlobalStateMachine().transfer(StateMachine.Actions.FINISH_SHOOT);
            timer.stop();
            timer.reset();
        }
    }
}
