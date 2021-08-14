package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.control.path.PathPoint

data class Pose(
    var p: Point = Point(),
    var h: Angle = Angle()
) {
    constructor(x: Double, y: Double, h: Angle) : this(Point(x, y), h)

    val x = p.x
    val y = p.y
    val cos = h.cos
    val sin = h.sin
    val hypot = p.hypot
    val copy get() = Pose(p, h)

    fun distance(p2: Pose) = p.distance(p2.p)
    fun distance(p2: PathPoint) = p.distance(p2.p)

    val toRawString = String.format("%.2f, %.2f, %.2f", x, y, h.raw)
    override fun toString() = String.format("%.2f, %.2f, %.2f", x, y, h.wrap().deg)
}
