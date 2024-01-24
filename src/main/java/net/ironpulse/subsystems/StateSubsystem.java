package net.ironpulse.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import net.ironpulse.RobotContainer;
import net.ironpulse.data.StateData;

import java.util.function.Consumer;

public class StateSubsystem implements Subsystem {
    private final RobotContainer robotContainer;
    private final Consumer<StateData> telemetryFunction;

    public StateSubsystem(RobotContainer robotContainer, Consumer<StateData> telemetryFunction) {
        CommandScheduler.getInstance().registerSubsystem(this);
        this.robotContainer = robotContainer;
        this.telemetryFunction = telemetryFunction;
    }

    @Override
    public void periodic() {
        telemetryFunction.accept(
                new StateData(robotContainer.getGlobalStateMachine().getCurrentState()));
    }
}
