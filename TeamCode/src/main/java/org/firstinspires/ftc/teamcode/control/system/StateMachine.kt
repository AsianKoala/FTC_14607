package org.firstinspires.ftc.teamcode.control.system

open class StateMachine(private val states: ArrayList<State>) {
    private var parallelStates = 0
    private var defaultKillCond: Boolean = false

    private val activeStates = HashMap<String, State>()
    private val deadStates = HashMap<String, State>()

    fun run() {
        if (!killCond()) {
            var done = 0
            activeStates.forEach {
                it.value.update()
                if (deadStates.containsKey(it.key)) {
                    it.value.onKill()
                    activeStates.remove(it.key)
                }

                if (it.value.killed) {
                    done++
                }
            }
            defaultKillCond = done < activeStates.size
        }
    }

    fun addState(name: String, state: State) {
        activeStates[name] = state
    }

    fun forceKillState(name: String) {
        if (activeStates.containsKey(name)) {
            deadStates[name] = activeStates[name]!!
        }
    }

    open fun killCond(): Boolean {
        return defaultKillCond
    }
}
