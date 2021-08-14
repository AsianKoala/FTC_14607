package org.firstinspires.ftc.teamcode.util

class MarkerNew(var name: String) {
    var start: Long = System.currentTimeMillis()
    var time = System.currentTimeMillis() - start

    fun start() {
        start = System.currentTimeMillis()
    }

    fun stop() {
        time = System.currentTimeMillis() - start
    }
}
