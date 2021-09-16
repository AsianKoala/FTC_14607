package org.firstinspires.ftc.teamcode.control.localization

import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.Point
import org.firstinspires.ftc.teamcode.util.math.Pose
import org.firstinspires.ftc.teamcode.util.math.TimePose
import kotlin.math.max

class Speedometer {
    private val pastPositions = ArrayList<TimePose>()
    private var bestIndex = 0

    fun update(position: Pose): Pose {
        return if(pastPositions.size == 0) {
            pastPositions.add(TimePose(position))
            Pose(Point.ORIGIN, Angle.EAST)
        } else {
            bestIndex = max(pastPositions.size - 4, 0)
            val ref = pastPositions[bestIndex]
            val dt = (System.currentTimeMillis() - ref.timestamp) / 1000.0
            val displacement = position.p - ref.pose.p
            val speeds = displacement / dt

            val dh = position.h - ref.pose.h
            val angvel = dh / dt

            pastPositions.add(TimePose(position))

            Pose(speeds, angvel)
        }

    }
}