package org.firstinspires.ftc.teamcode.control.controllers

import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.control.path.waypoints.LockedWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.StopWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import org.firstinspires.ftc.teamcode.hardware.Azusa
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.MathUtil.circleLineIntersection
import org.firstinspires.ftc.teamcode.util.math.MathUtil.clip
import org.firstinspires.ftc.teamcode.util.math.MathUtil.clipIntersection
import org.firstinspires.ftc.teamcode.util.math.MathUtil.toRadians
import org.firstinspires.ftc.teamcode.util.math.Point
import org.firstinspires.ftc.teamcode.util.math.Pose
import kotlin.math.PI
import kotlin.math.absoluteValue

object PurePursuitController {
//    private val mins = Pose(0.11, 0.09, 0.11)

    private fun relVals(curr: Pose, target: Waypoint): Pose {
        val d = (curr.p - target.p).hypot
        val rh = (target.p - curr.p).atan2 - curr.h
        return Pose(Point(-d * rh.sin, d * rh.cos), rh)
    }

    fun goToPosition(azusa: Azusa, target: Waypoint) {
        val (x, y) = target.p.dbNormalize
        azusa.azuTelemetry.fieldOverlay()
                .setStroke("yellow")
                .strokeCircle(x, y, 3.0)


        var movementPoint = Point.ORIGIN

        val pointDeltas = relVals(azusa.currPose, target).p

        val relativeAngle = getDeltaH(azusa.currPose, target)
        var turnPower = relativeAngle / 30.0.toRadians


        movementPoint.x = (pointDeltas.x / (pointDeltas.x.absoluteValue + pointDeltas.y.absoluteValue))
        movementPoint.y = (pointDeltas.y / (pointDeltas.x.absoluteValue + pointDeltas.y.absoluteValue))

        movementPoint.x *= pointDeltas.x.absoluteValue / 12.0
        movementPoint.y *= pointDeltas.y.absoluteValue / 12.0

        movementPoint = movementPoint.clip(1.0)



        movementPoint.x *= (pointDeltas.x / 2.5).clip(1.0)
        movementPoint.y *= (pointDeltas.y / 2.5).clip(1.0)

        turnPower *= (relativeAngle.absoluteValue / 3.0.toRadians).clip(1.0).clip(1.0)


        if(azusa.currPose.distance(target) < 3.5) {
            turnPower = 0.0
        }

        var errorTurnScale = Range.clip(1.0-(relativeAngle / 45.0.toRadians), 0.4, 1.0)

        if(turnPower < 0.0001) {
            errorTurnScale = 1.0
        }

        movementPoint *= errorTurnScale


        azusa.driveTrain.powers = Pose(movementPoint, Angle(turnPower, AngleUnit.RAW))
    }

    private fun getDeltaH(curr: Pose, target: Waypoint): Double {
        return if(target is LockedWaypoint) {
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
        val (x, y) = circleLineIntersection(clip    , start.p, end.p, end.followDistance)
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

