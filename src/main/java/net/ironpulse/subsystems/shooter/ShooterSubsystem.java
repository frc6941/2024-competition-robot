package net.ironpulse.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static net.ironpulse.Constants.ShooterConstants.*;

@Getter
public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();


    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        setDefaultCommand(run(() -> io.setShooterVoltage(shooterConstantVoltage)));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        if (inputs.homed) return;
        if (inputs.armSupplyCurrent.magnitude() > armZeroCurrent.magnitude()) {
            io.setArmVoltage(Volts.of(0));
            io.setArmHome(Radians.of(0));
            getIo().setHomed(true);
            return;
        }
        io.setArmVoltage(armZeroVoltage);
    }
}
