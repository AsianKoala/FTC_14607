package org.firstinspires.ftc.teamcode.control.path

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

object PurePursuitController {

    private fun relVals(curr: Pose, target: Waypoint): Point {
        val d = (curr.p - target.p).hypot
        val rh = (target.p - curr.p).atan2 - curr.h
        return Point(-d * rh.sin, d * rh.cos)
    }

    fun goToPosition(azusa: Azusa, target: Waypoint) {
        val (x, y) = target.p.dbNormalize
        azusa.azuTelemetry.fieldOverlay()
            .setStroke("yellow")
            .strokeCircle(x, y, 3.0)

        val relTarget = relVals(azusa.currPose, target)

        val sumAbs = relTarget.x.absoluteValue + relTarget.y.absoluteValue

        val movementPowers = (relTarget / sumAbs)
        movementPowers.x *= relTarget.x.absoluteValue / 18.0
        movementPowers.y *= relTarget.y.absoluteValue / 18.0


        if(target is StopWaypoint) {
            movementPowers.x = relTarget.x / 14.0
            movementPowers.y = relTarget.y / 14.0
        }


        val deltaH = getDeltaH(azusa.currPose, target)
        var turnPower = deltaH / 60.0.toRadians

        if(azusa.currPose.distance(target) < 3) {
            turnPower = 0.0
        }

        azusa.driveTrain.powers = Pose(movementPowers, Angle(turnPower, AngleUnit.RAW))
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

        azusa.azuTelemetry.addData("followpoint", followPoint.p)
        azusa.azuTelemetry.fieldOverlay()
            .setStroke("white")
            .strokeLine(azusa.currPose.p.dbNormalize.x, azusa.currPose.p.dbNormalize.y, followPoint.p.dbNormalize.x, followPoint.p.dbNormalize.y)

        goToPosition(azusa, followPoint)
    }
}
