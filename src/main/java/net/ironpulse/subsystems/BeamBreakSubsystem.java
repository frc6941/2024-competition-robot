package net.ironpulse.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.Getter;
import net.ironpulse.RobotContainer;
import net.ironpulse.drivers.BeamBreak;

import static net.ironpulse.Constants.BeamBreakConstants.*;

@Getter
public class BeamBreakSubsystem implements Subsystem {
    private final BeamBreak intakerBeamBreak;

    private final BeamBreak indexerBeamBreak1;

    private final BeamBreak indexerBeamBreak2;

    private final RobotContainer robotContainer;

    public BeamBreakSubsystem(RobotContainer robotContainer) {
        CommandScheduler.getInstance().registerSubsystem(this);
        this.robotContainer = robotContainer;
        intakerBeamBreak = new BeamBreak(INTAKER_BEAM_BREAK_ID);
        indexerBeamBreak1 = new BeamBreak(INDEXER_BEAM_BREAK_1_ID);
        indexerBeamBreak2 = new BeamBreak(INDEXER_BEAM_BREAK_2_ID);
    }

    @Override
    public void periodic() {
        if (!intakerBeamBreak.get() && indexerBeamBreak1.get() && indexerBeamBreak2.get()) {
            robotContainer.getGlobalState().transfer(RobotContainer.Actions.FINISH_INTAKE);
            return;
        }

        if (!intakerBeamBreak.get() && !indexerBeamBreak1.get() && !indexerBeamBreak2.get()) {
            robotContainer.getGlobalState().transfer(RobotContainer.Actions.FINISH_SHOOT);
        }
    }
}
