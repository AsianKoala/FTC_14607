package org.firstinspires.ftc.teamcode.control.system

class StateMachineBuilder {
    private val states = ArrayList<State>()

    fun addState(state: State): StateMachineBuilder {
        states.add(state)
        return this
    }

    fun build(): StateMachine {
        val copyList = ArrayList<State>()
        for (state in states) {
            copyList.add(state)
        }
        return StateMachine(states)
    }
}
