package org.firstinspires.ftc.teamcode.control.localization

import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.MathUtil.epsilonEquals
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose

object Speedometer {
    private var lastUpdateTime = 0.0
    private var currSpeedX = 0.0
    private var currSpeedY = 0.0

    var timeBetweenUpdates = 25
    var xDistTraveled = 0.0
    var yDistTraveled = 0.0

    var lastAngle = Angle(0.0, Angle.Unit.RAD)
    var angularVel = Angle(0.0, Angle.Unit.RAD)

    fun update(h: Angle): Pose {
        val currTime = System.currentTimeMillis()

        if (xDistTraveled epsilonEquals 0.0 && yDistTraveled epsilonEquals 0.0 && angularVel.angle epsilonEquals 0.0)
            return Pose(Point(xDistTraveled, yDistTraveled), angularVel)

        if (currTime - lastUpdateTime > timeBetweenUpdates) {
            val elapsed: Double = (currTime - lastUpdateTime) / 1000.0
            val speedY = yDistTraveled / elapsed
            val speedX = xDistTraveled / elapsed

            currSpeedY = speedY
            currSpeedX = speedX

            angularVel = (h - lastAngle) / elapsed
            lastAngle = h

            yDistTraveled = 0.0
            xDistTraveled = 0.0
            lastUpdateTime = currTime.toDouble()
        }
        return Pose(Point(xDistTraveled, yDistTraveled), angularVel)
    }
}
