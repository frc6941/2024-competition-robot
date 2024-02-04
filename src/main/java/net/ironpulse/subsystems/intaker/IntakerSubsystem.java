package net.ironpulse.subsystems.intaker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

public class IntakerSubsystem extends SubsystemBase {
    private final IntakerIO io;
    private final IntakerIOInputsAutoLogged inputs = new IntakerIOInputsAutoLogged();

    public IntakerSubsystem(IntakerIO io) {
        this.io = io;
        setDefaultCommand(run(() -> io.setIntakeVoltage(Volts.of(0))));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intaker", inputs);
    }
}
