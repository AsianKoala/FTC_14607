package org.firstinspires.ftc.teamcode.control.system

abstract class State {
    protected abstract fun run()

    open val name: String = ""

    protected open fun onStart() {}
    open fun onKill() {}

    open val killCond: Boolean = true
    protected open val runCond: Boolean = true

    private var started: Boolean = false
    var killed: Boolean = false
        private set

    fun update() {
        if (!started && runCond) {
            onStart()
            started = true
        }

        if (!killed) {
            if (runCond)
                run()

            if (killCond) {
                onKill()
                killed = true
            }
        }
    }
}
