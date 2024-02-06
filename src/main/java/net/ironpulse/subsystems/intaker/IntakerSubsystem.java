package net.ironpulse.subsystems.intaker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

@Getter
public class IntakerSubsystem extends SubsystemBase {
    private final IntakerIO io;
    private final IntakerIOInputsAutoLogged inputs = new IntakerIOInputsAutoLogged();

    public IntakerSubsystem(IntakerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intaker", inputs);
    }
}
