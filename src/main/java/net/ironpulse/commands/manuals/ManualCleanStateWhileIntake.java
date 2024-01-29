package net.ironpulse.commands.manuals;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.RobotContainer;
import net.ironpulse.state.StateMachine;

public class ManualCleanStateWhileIntake extends Command{
    
    private final RobotContainer robotContainer;

    public ManualCleanStateWhileIntake(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    @Override
    public void execute() {
        
        robotContainer.getGlobalStateMachine().setCurrentState(StateMachine.States.IDLE);
    }
}
