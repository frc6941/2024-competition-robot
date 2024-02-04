package net.ironpulse.subsystems.beambreak;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

@Getter
public class BeamBreakSubsystem extends SubsystemBase {
    private final BeamBreakIO io;
    private final BeamBreakIOInputsAutoLogged inputs = new BeamBreakIOInputsAutoLogged();

    public BeamBreakSubsystem(BeamBreakIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Beam Break", inputs);
    }
}
