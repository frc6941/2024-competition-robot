package net.ironpulse.commands.manuals;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.RobotContainer;
import net.ironpulse.state.StateMachine;
import net.ironpulse.subsystems.IntakerSubsystem;

public class ManualCleanStateWhileIntake extends Command{
    
    private final RobotContainer robotContainer;

    public ManualCleanStateWhileIntake(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    @Override
    public void execute() {
        
        robotContainer.getGlobalStateMachine().transfer(StateMachine.Actions.INTERRUPT_INTAKE);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
