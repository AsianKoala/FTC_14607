package org.firstinspires.ftc.teamcode.util.math

import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint

data class Pose(
        var p: Point,
        var h: Angle
) {
    val x get() = p.x
    val y get() = p.y
    val cos get() = h.cos
    val sin get() = h.sin
    val hypot get() = p.hypot
    val copy get() = Pose(p, h)

    fun distance(p2: Pose) = p.distance(p2.p)
    fun distance(p2: Waypoint) = p.distance(p2.p)

    val toRawString = String.format("%.2f, %.2f, %.2f", x, y, h.raw)
    override fun toString() = String.format("%.2f, %.2f, %.2f", x, y, h.wrap().deg)
}
