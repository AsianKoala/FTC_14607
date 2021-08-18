package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.util.Angle

open class LockedWaypoint(
    x: Double,
    y: Double,
    followDistance: Double,
    val h: Angle,
    func: Functions.Function? = null
) : Waypoint(x, y, followDistance, func) {

    override val copy: Waypoint get() = LockedWaypoint(x, y, followDistance, h, func)
    override fun toString(): String {
        return String.format(
            "%.1f, %.1f, %.1f, %.1f, %s",
            x,
            y,
            followDistance,
            h,
            func
        )
    }
}
