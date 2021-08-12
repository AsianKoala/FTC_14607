package org.firstinspires.ftc.teamcode.control.localization

import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.MathUtil
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose

class Speedometer {
    private var lastUpdateTime = 0.0

    private var lastAngle = Angle(0.0, Angle.Unit.RAD)
    private var angularVel = Angle(0.0, Angle.Unit.RAW)

    fun update(h: Angle): Pose {
        val currTime = System.currentTimeMillis() / 1000.0
        val dt = currTime - lastUpdateTime
        lastUpdateTime = currTime

        val speeds = deltas / dt

        angularVel = (h - lastAngle) / dt
        lastAngle = h

        deltas = Point()

        return Pose(MathUtil.rotatePoint(speeds, h), angularVel)
    }

    companion object {
        var deltas = Point()
    }
}
