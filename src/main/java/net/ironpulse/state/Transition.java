package net.ironpulse.state;

import edu.wpi.first.wpilibj2.command.Command;
import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class Transition {
    private StateMachine.States currentState;

    private StateMachine.States nextState;

    /**
     * The action required to do this transition
     */
    private StateMachine.Actions action;

    /**
     * The command that will be executed after this transition
     */
    private Command command;
}