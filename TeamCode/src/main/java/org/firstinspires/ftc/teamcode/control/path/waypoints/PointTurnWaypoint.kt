package org.firstinspires.ftc.teamcode.control.path.waypoints

import org.firstinspires.ftc.teamcode.control.path.funcs.Func
import org.firstinspires.ftc.teamcode.util.math.Angle

class PointTurnWaypoint(
    x: Double,
    y: Double,
    followDistance: Double,
    h: Angle,
    var dh: Angle,
    func: Func? = null
) : LockedWaypoint(x, y, followDistance, h, func) {

    override val copy: Waypoint get() = PointTurnWaypoint(x, y, followDistance, h, dh, func)
    override fun toString(): String {
        return String.format(
            "%.1f, %.1f, %.1f, %s, %s, %s",
            x,
            y,
            followDistance,
            h,
            dh,
            func
        )
    }
}
