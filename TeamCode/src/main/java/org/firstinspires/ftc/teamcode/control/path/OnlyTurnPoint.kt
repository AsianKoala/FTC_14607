package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.control.system.Functions
import org.firstinspires.ftc.teamcode.util.Angle

class OnlyTurnPoint(
    signature: String,
    x: Double,
    y: Double,
    followDistance: Double,
    h: Angle,
    var dh: Angle,
    func: Functions.Function? = null
) : LockedPathPoint(signature, x, y, followDistance, h, func) {

    override val copy: PathPoint get() = OnlyTurnPoint(signature, x, y, followDistance, h, dh, func)
    override fun toString(): String {
        return String.format(
            "%s, %.1f, %.1f, %.1f, %.1f, %.1f, %s",
            signature,
            x,
            y,
            followDistance,
            h,
            dh,
            func
        )
    }
}
