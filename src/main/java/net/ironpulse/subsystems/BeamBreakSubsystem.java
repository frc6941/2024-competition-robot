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

    private final BeamBreak indexerBeamBreak1;

    private final BeamBreak indexerBeamBreak2;

    private final RobotContainer robotContainer;
    private final Consumer<BeamBreakData> telemetryFunction;

    private final Timer timer = new Timer();

    public BeamBreakSubsystem(RobotContainer robotContainer, Consumer<BeamBreakData> telemetryFunction) {
        CommandScheduler.getInstance().registerSubsystem(this);
        this.telemetryFunction = telemetryFunction;
        this.robotContainer = robotContainer;
        intakerBeamBreak = new BeamBreak(INTAKER_BEAM_BREAK_ID);
        indexerBeamBreak1 = new BeamBreak(INDEXER_BEAM_BREAK_1_ID);
        indexerBeamBreak2 = new BeamBreak(INDEXER_BEAM_BREAK_2_ID);
    }

    @Override
    public void periodic() {
        telemetryFunction.accept(
                new BeamBreakData(
                        intakerBeamBreak.get(),
                        indexerBeamBreak1.get(),
                        indexerBeamBreak2.get()
                )
        );
        if (!intakerBeamBreak.get() && indexerBeamBreak1.get() && indexerBeamBreak2.get()) {
            robotContainer.getGlobalState().transfer(StateMachine.Actions.FINISH_INTAKE);
            return;
        }

        if (!intakerBeamBreak.get() && !indexerBeamBreak1.get() && !indexerBeamBreak2.get()) {
            timer.start();
            if (!timer.hasElapsed(0.5)) return;
            robotContainer.getGlobalState().transfer(StateMachine.Actions.FINISH_SHOOT);
            timer.stop();
            timer.reset();
        }
    }
}
