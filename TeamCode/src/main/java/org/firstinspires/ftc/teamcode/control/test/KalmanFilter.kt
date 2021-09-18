package org.firstinspires.ftc.teamcode.control.test

class KalmanFilter(var x: Double, val Q: Double, val R: Double, var P: Double, var K: Double) {

    private var xprev = x
    private var pprev = P
    private var u = 0.0
    private var z = 0.0

    fun update(inp1: Double, inp2: Double): Double {
        u = inp1
        x = xprev + u
        P = pprev + Q
        K = P / (P + R)
        z = inp2
        x += K * (z - x)
        P *= (1 - K)

        xprev = x
        pprev = P
        return x
    }
}
