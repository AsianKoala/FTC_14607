package org.firstinspires.ftc.teamcode.control.controllers

import org.firstinspires.ftc.teamcode.control.path.waypoints.LockedWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.StopWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import org.firstinspires.ftc.teamcode.hardware.Azusa
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.MathUtil.circleLineIntersection
import org.firstinspires.ftc.teamcode.util.math.MathUtil.clipIntersection
import org.firstinspires.ftc.teamcode.util.math.MathUtil.toRadians
import org.firstinspires.ftc.teamcode.util.math.Point
import org.firstinspires.ftc.teamcode.util.math.Pose
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.max

object PurePursuitController {
//    private val mins = Pose(0.11, 0.09, 0.11)

    private fun relVals(curr: Pose, target: Waypoint): Pose {
        val d = (curr.p - target.p).hypot
        val rh = (target.p - curr.p).atan2 - curr.h
        return Pose(Point(-d * rh.sin, d * rh.cos), rh)
    }

    fun goToPosition(azusa: Azusa, target: Waypoint) {
        val pointDeltas = relVals(azusa.currPose, target).p / 12.0
        val dh = getDeltaH(azusa.currPose, target) / 60.0.toRadians

        if(target !is StopWaypoint) {
            val totalAbs = azusa.driveTrain.powers.x.absoluteValue + azusa.driveTrain.powers.y.absoluteValue
            if (totalAbs != 0.0)
                azusa.driveTrain.powers.p /= totalAbs
        }

        val (x, y) = target.p.dbNormalize
        azusa.azuTelemetry.fieldOverlay().fillCircle(x, y, 3.0)

        azusa.driveTrain.powers = Pose(pointDeltas, Angle(dh, AngleUnit.RAW))
    }

    private fun getDeltaH(curr: Pose, target: Waypoint): Double {
        return if (target is LockedWaypoint) {
            (target.h - curr.h).wrap().raw
        } else {
            val forward = (target.p - curr.p).atan2
            val back = forward + Angle(PI, AngleUnit.RAD)
            val angleToForward = (forward - curr.h).wrap()
            val angleToBack = (back - curr.h).wrap()
            val autoAngle = if (angleToForward.abs < angleToBack.abs) forward else back
            (autoAngle - curr.h).wrap().raw
        }
    }

    fun followPath(azusa: Azusa, start: Waypoint, end: Waypoint) {
        val clip: Point = clipIntersection(start.p, end.p, azusa.currPose.p)
        val (x, y) = circleLineIntersection(clip, start.p, end.p, end.followDistance)
        val followPoint = end.copy
        followPoint.x = x
        followPoint.y = y
        azusa.azuTelemetry.fieldOverlay()
            .setFill("white")
            .fillCircle(followPoint.p.dbNormalize.x, followPoint.p.dbNormalize.y, 2.0)
        goToPosition(azusa, followPoint)
    }
}
