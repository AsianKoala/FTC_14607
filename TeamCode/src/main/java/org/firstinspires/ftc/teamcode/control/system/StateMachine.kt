package org.firstinspires.ftc.teamcode.control.system

// todo
class StateMachine(val states: ArrayList<State>) {
    var curr = 0
    val running get() = curr < states.size

    fun run() {
        if (running) {
            println("curr: $curr")
            val currState = states[curr]
            println("current state: ${currState.name}")
            currState.run()
            if (currState.skipCondition) {
                curr++
            }
        }
    }
}
