package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.control.system.Functions
import org.firstinspires.ftc.teamcode.util.Angle

class StopPathPoint(
    signature: String,
    x: Double,
    y: Double,
    followDistance: Double,
    h: Angle,
    func: Functions.Function? = null
) : LockedPathPoint(signature, x, y, followDistance, h, func) {

    override val copy: PathPoint get() = StopPathPoint(signature, x, y, followDistance, h, func)
    override fun toString(): String {
        return String.format(
            "%s, %.1f, %.1f, %.1f, %.1f, %s",
            signature,
            x,
            y,
            followDistance,
            h,
            func
        )
    }
}
