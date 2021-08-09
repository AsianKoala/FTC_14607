package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.control.path.PathPoint
import kotlin.math.absoluteValue
import kotlin.math.sign

data class Pose(
    var p: Point = Point(),
    var h: Angle = Angle()
) {
    constructor(x: Double, y: Double, h: Double) : this(Point(x, y), Angle(h))
    constructor(p: Point, h: Double = 0.0) : this(p.x, p.y, h)
    constructor(p: Pose2d) : this(p.x, p.y, p.heading)

    val x = p.x
    val y = p.y
    val cos = h.cos
    val sin = h.sin
    val hypot = p.hypot
    val copy get() = Pose(p, h)

    fun clipAbs(max: Double): Pose {
        val ret = copy
        if (x.absoluteValue > max) {
            ret.p.x = x.sign * max
        }
        if (y.absoluteValue > max) {
            ret.p.y = y.sign * max
        }
        if (h.abs > max) {
            ret.h = Angle(h.sign * max)
        }
        return ret
    }

    fun distance(p2: Pose) = p.distance(p2.p)
    fun distance(p2: PathPoint) = p.distance(p2.p)

    val toRawString = String.format("(%.2f, %.2f, %.2f", x, y, h.raw)
    override fun toString() = String.format("(%.2f, %.2f, %.2f", x, y, h.wrap().deg)
}
