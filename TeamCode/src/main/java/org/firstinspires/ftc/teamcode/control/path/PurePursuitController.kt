package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.control.path.waypoints.LockedWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import org.firstinspires.ftc.teamcode.hardware.OldAzusa
import org.firstinspires.ftc.teamcode.util.math.*
import org.firstinspires.ftc.teamcode.util.math.MathUtil.toRadians
import kotlin.math.PI

object PurePursuitController {

    fun relVals(curr: Pose, target: Point): Point {
        val d = (curr.p - target).hypot
        val rh = (target - curr.p).atan2 - curr.h
        return Point(-d * rh.sin, d * rh.cos)
    }

    fun goToPosition(azusa: OldAzusa, target: Waypoint) {
        val (x, y) = target.p.dbNormalize
        azusa.azuTelemetry.fieldOverlay()
            .setStroke("purple")
            .strokeCircle(x, y, 1.0)

        val relTarget = relVals(azusa.currPose, target.p)

        val movementPowers = (relTarget / 12.0)

        val deltaH = getDeltaH(azusa.currPose, target)
        val turnPower = deltaH / 90.0.toRadians

        azusa.driveTrain.powers = Pose(movementPowers, Angle(turnPower, AngleUnit.RAW))
    }

    fun getDeltaH(curr: Pose, target: Waypoint): Double {
        return if (target is LockedWaypoint) {
            (target.h - curr.h).wrap().angle
        } else {
            val forward = (target.p - curr.p).atan2
            val back = forward + Angle(PI, AngleUnit.RAD)
            val angleToForward = (forward - curr.h).wrap()
            val angleToBack = (back - curr.h).wrap()
            val autoAngle = if (angleToForward.abs < angleToBack.abs) forward else back
            (autoAngle - curr.h).wrap().angle
        }
    }
}
