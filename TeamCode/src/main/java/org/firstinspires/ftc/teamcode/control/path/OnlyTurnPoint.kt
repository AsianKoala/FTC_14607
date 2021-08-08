package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.control.system.Functions
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.Point

class OnlyTurnPoint(
    signature: String,
    p: Point,
    followDistance: Double,
    h: Angle,
    var dh: Angle,
    func: Functions.Function?
) : LockedPathPoint(signature, p, followDistance, h, func) {
    constructor(
        signature: String,
        x: Double,
        y: Double,
        followDistance: Double,
        h: Angle,
        dh: Angle,
        func: Functions.Function? = null
    ) : this(signature, Point(x, y), followDistance, h, dh, func)

    override val copy: PathPoint = OnlyTurnPoint(signature, p, followDistance, h, dh, func)
    override fun toString(): String {
        return String.format(
            "%s, %.1f, %.1f, %.1f, %.1f, %.1f, %s",
            signature,
            p.x,
            p.y,
            followDistance,
            h,
            dh,
            func.toString()
        )
    }
}
