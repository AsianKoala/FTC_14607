package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.control.system.Functions
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose

// id love for this to be a dataclass but yeah sucks to suck
open class PathPoint constructor(
    var signature: String,
    var p: Point,
    var followDistance: Double,
    var func: Functions.Function? = null
) {
    constructor(
        signature: String,
        x: Double,
        y: Double,
        followDistance: Double,
        func: Functions.Function? = null
    ) : this(signature, Point(x, y), followDistance, func)

    open val copy = PathPoint(signature, p, followDistance, func)
    fun distance(p2: PathPoint) = p.distance(p2.p)
    fun distance(p2: Pose) = p.distance(p2.p)
    override fun toString(): String {
        return String.format(
            "%s, %.1f, %.1f, %.1f, %s",
            signature,
            p.x,
            p.y,
            followDistance,
            func.toString()
        )
    }
}
