package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.control.path.funcs.Functions
import org.firstinspires.ftc.teamcode.control.path.waypoints.LockedWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.PointTurnWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.StopWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import org.firstinspires.ftc.teamcode.hardware.Azusa
import org.firstinspires.ftc.teamcode.util.math.*
import org.firstinspires.ftc.teamcode.util.math.MathUtil.circleLineIntersection
import org.firstinspires.ftc.teamcode.util.math.MathUtil.clipIntersection
import org.firstinspires.ftc.teamcode.util.math.MathUtil.toRadians
import kotlin.math.PI

object PurePursuitController {

    private fun relVals(curr: Pose, target: Waypoint): Point {
        val d = (curr.p - target.p).hypot
        val rh = (target.p - curr.p).atan2 - curr.h
        return Point(-d * rh.sin, d * rh.cos)
    }

    fun goToPosition(azusa: Azusa, target: Waypoint) {
        val (x, y) = target.p.dbNormalize
        azusa.azuTelemetry.fieldOverlay()
            .setStroke("purple")
            .strokeCircle(x, y, 1.0)

        val relTarget = relVals(azusa.currPose, target)

        val movementPowers = (relTarget / 12.0)

        val deltaH = getDeltaH(azusa.currPose, target)
        val turnPower = deltaH / 90.0.toRadians

        azusa.driveTrain.powers = Pose(movementPowers, Angle(turnPower, AngleUnit.RAW))
    }

    private fun getDeltaH(curr: Pose, target: Waypoint): Double {
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

    fun followPath(azusa: Azusa, path: Path) {
        val currPose = azusa.currPose

        val target = path.target

        var skip: Boolean
        do {
            skip = when (target) {
                is StopWaypoint -> currPose.distance(target) < 0.8 && MathUtil.angleThresh(currPose.h, target.h, target.dh)
                is PointTurnWaypoint -> MathUtil.angleThresh(currPose.h, target.h, target.dh)
                else -> currPose.distance(target) < target.followDistance
            }

            val startAction = path.start.func
            if (startAction is Functions.RepeatFunction) {
                startAction.run(azusa, path)
            } else if (startAction is Functions.LoopUntilFunction) {
                skip = startAction.run(azusa, path)
            }

            if (skip) {
                println(path.incWaypoint())

                val currAction = path.start.func
                if (currAction is Functions.SimpleFunction) {
                    currAction.run(azusa, path)
                }
            }
        } while (skip && !path.isFinished)

        val nStart = path.start
        val nEnd = path.target

        val clip: Point = clipIntersection(nStart.p, nEnd.p, azusa.currPose.p)
        val (x, y) = circleLineIntersection(clip, nStart.p, nEnd.p, nEnd.followDistance)
        val followPoint = nEnd.copy
        followPoint.x = x
        followPoint.y = y

        azusa.azuTelemetry.addData("followpoint", followPoint.p)
        azusa.azuTelemetry.fieldOverlay()
            .setStroke("white")
            .strokeLine(azusa.currPose.p.dbNormalize.x, azusa.currPose.p.dbNormalize.y, followPoint.p.dbNormalize.x, followPoint.p.dbNormalize.y)

        if ((target is StopWaypoint && azusa.currPose.distance(target) < target.followDistance) || target is PointTurnWaypoint) {
            goToPosition(azusa, nEnd)
        } else {
            goToPosition(azusa, followPoint)
        }
    }
}
