package net.ironpulse.data;

import net.ironpulse.state.StateMachine;

public record StateData(
        StateMachine.States currentState
) {
}
