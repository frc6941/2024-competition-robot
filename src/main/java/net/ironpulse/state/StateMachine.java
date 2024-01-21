package net.ironpulse.state;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lombok.Getter;

import java.util.List;
import java.util.Optional;

/**
 * This class is a State Machine framework for easier use of FSM (Finite State Machine)
 */
public class StateMachine {
    private final List<Transition> transitions;

    @Getter
    private States currentState;

    public StateMachine(States initialState, List<Transition> transitions) {
        currentState = initialState;
        this.transitions = transitions;
    }

    /**
     * Do an action to trigger a specific transition
     * @param action The action that is going to be done
     */
    public synchronized void transfer(Actions action) {
        var transition = transitions
                .stream()
                .filter(trans ->
                        trans.getAction() == action
                                && trans.getCurrentState() == currentState)
                .findFirst();
        if (transition.isEmpty()) return;
        currentState = transition.get().getNextState();
        Optional.ofNullable(transition.get().getCommand())
                .ifPresent(command ->
                        CommandScheduler
                                .getInstance()
                                .schedule(command)
                );
    }

    public enum States {
        IDLE, INTAKING, PENDING, SHOOTING
    }

    public enum Actions {
        INTAKE, FINISH_INTAKE, SHOOT, FINISH_SHOOT, INTERRUPT_INTAKE, INTERRUPT_SHOOT
    }
}