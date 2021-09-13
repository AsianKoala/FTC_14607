package org.firstinspires.ftc.teamcode.control.test

class KalmanFilter {
    private var x = 0.0
    private val Q = 0.1
    private val R = 0.4
    private var p = 1.0
    private var K = 1.0

    private var xprev = x
    private var pprev = p
    private  var u = 0.0
    private var z = 0.0

    fun update(inp1: Double, inp2: Double): Double {
        u = inp1
        x = xprev + u
        p = pprev + Q
        K = p / (p + R)
        z = inp2
        x += K * (z - x)
        p *= (1 - K)

        xprev = x
        pprev = p
        return x
    }
}