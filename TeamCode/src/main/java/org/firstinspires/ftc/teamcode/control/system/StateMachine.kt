package org.firstinspires.ftc.teamcode.control.system

open class StateMachine(private val states: ArrayList<State>) {
    private var parallelStates = 0
    private var defaultKillCond: Boolean = false

    fun run() {
        if(!killCond()) {
            var kilamt = 0
            for(state in states) {
                state.update()
                if(state.killed)
                    kilamt++
            }
            defaultKillCond = kilamt + parallelStates >= states.size
        }
    }

    open fun killCond(): Boolean {
        return defaultKillCond
    }
}
