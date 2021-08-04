package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.control.system.Functions
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.Point

class StopPathPoint(
    signature: String,
    p: Point,
    followDistance: Double,
    var h: Angle,
    func: Functions.Function?
) : PathPoint(signature, p, followDistance, func) {
    constructor(
        signature: String,
        x: Double,
        y: Double,
        followDistance: Double,
        h: Angle,
        func: Functions.Function? = null
    ): this(signature, Point(x,y), followDistance, h, func)
    override val copy: PathPoint = StopPathPoint(signature, p, followDistance, h, func)
    override fun toString(): String {
        return String.format("%s, %.1f, %.1f, %.1f, %.1f, %s", signature, p, followDistance, h, func.toString())
    }
}
