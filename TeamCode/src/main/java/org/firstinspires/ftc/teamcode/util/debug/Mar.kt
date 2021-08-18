package org.firstinspires.ftc.teamcode.util.debug

class Mar {
    var start = 0.0
    var time = System.currentTimeMillis() - start
        private set

    fun start() {
        start = System.currentTimeMillis().toDouble()
    }

    fun stop() {
        time = System.currentTimeMillis() - start
    }
}
