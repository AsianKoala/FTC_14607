package org.firstinspires.ftc.teamcode.control.test

import kotlin.math.abs
import kotlin.math.sign

class PIDFController(val kp: Double, val ki: Double, val kd: Double, val kv: Double, val ka: Double, val ks: Double) {

    var target = 0.0
    var targetVel = 0.0
    var targetAccel = 0.0

    var errorsum = 0.0
    var lasterror = 0.0

    var lastupdate: Long = 0

    var bounded = false
    var upperbound = 1.0
    var lowerbound = -1.0

    fun setTargets(t: Double, tv: Double, ta: Double) {
        target = t
        targetVel = tv
        targetAccel = ta
    }

    fun setBounds(upper: Double, lower: Double) {
        bounded = true
        upperbound = upper
        lowerbound = lower
    }

    fun update(position: Double, refvel: Double?): Double {
        val error = target - position
        val timestamp = System.currentTimeMillis()

        return if (lastupdate.toInt() == 0) {
            lasterror = error
            lastupdate = timestamp
            0.0
        } else {
            val dt = timestamp - lastupdate
            val deriv = (error - lasterror) / dt
            errorsum += error * dt

            lasterror = error
            lastupdate = timestamp

            val baseoutput = kp * error + ki * errorsum + kd * deriv + (refvel ?: targetVel) * kv + targetAccel * ka
            val output = if (abs(baseoutput) < 0.0000001) 0.0 else baseoutput + baseoutput.sign * ks

            if (bounded) {
                when {
                    output < lowerbound -> lowerbound
                    output > upperbound -> upperbound
                    else -> output
                }
            } else {
                output
            }
        }
    }
}
