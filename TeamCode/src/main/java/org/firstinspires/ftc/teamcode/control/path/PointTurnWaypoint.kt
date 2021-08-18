package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.util.Angle

class PointTurnWaypoint(
    signature: String,
    x: Double,
    y: Double,
    followDistance: Double,
    h: Angle,
    var dh: Angle,
    func: Functions.Function? = null
) : LockedWaypoint(signature, x, y, followDistance, h, func) {

    override val copy: Waypoint get() = PointTurnWaypoint(signature, x, y, followDistance, h, dh, func)
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