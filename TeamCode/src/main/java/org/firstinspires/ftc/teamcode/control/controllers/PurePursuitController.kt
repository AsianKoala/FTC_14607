package org.firstinspires.ftc.teamcode.control.controllers

import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.control.path.LockedPathPoint
import org.firstinspires.ftc.teamcode.control.path.PathPoint
import org.firstinspires.ftc.teamcode.control.path.StopPathPoint
import org.firstinspires.ftc.teamcode.hardware.Azusa
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.MathUtil.circleLineIntersection
import org.firstinspires.ftc.teamcode.util.MathUtil.clipIntersection
import org.firstinspires.ftc.teamcode.util.MathUtil.toRadians
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose
import kotlin.math.PI
import kotlin.math.pow
import kotlin.math.sign

object PurePursuitController {
//    private val mins = Pose(0.11, 0.09, 0.11)

    private fun relVals(curr: Pose, target: PathPoint): Pose {
        val d = (curr.p - target.p).hypot
        val rh = (target.p - curr.p).atan2 - curr.h
        return Pose(Point(-d * rh.sin, d * rh.cos), rh)
    }

    fun goToPosition(azusa: Azusa, target: PathPoint) {
        val currPose = azusa.currPose

        if (target !is StopPathPoint || currPose.distance(target) > 18) {
            val deltas = relVals(currPose, target).p / 12.0
            val dh = getDeltaH(currPose, target) / 35.0.toRadians

            azusa.driveTrain.powers = Pose(deltas, Angle(dh, Angle.Unit.RAW))
        } else if (azusa.currVel.hypot > 8.0 && currPose.distance(target) > 6) {
            val clip = circleLineIntersection(target.p, currPose.p, target.p, 8.0)
            val deltas = relVals(currPose, PathPoint(x = clip.x, y = clip.y)).p
            val dh = getDeltaH(currPose, target) / 35.0.toRadians

            azusa.driveTrain.powers = Pose(deltas, Angle(dh, Angle.Unit.RAW))
        } else {
            val deltas = relVals(currPose, target).p
            val dh = getDeltaH(currPose, target)

            azusa.driveTrain.powers = Pose(
                Point(
                    deltas.x.sign * deltas.x.pow(1.0 / 6.0),
                    deltas.y.sign * deltas.y.pow(1.0 / 6.0)
                ),
                Angle(dh.sign * dh.pow(1.0 / 6.0), Angle.Unit.RAW)
            )
        }
    }

    fun gunToPosition(azusa: Azusa, target: PathPoint, moveSpeed: Double) {
        val pointDeltas = relVals(azusa.currPose, target).p / 12.0
        var dh = getDeltaH(azusa.currPose, target) / 60.0.toRadians

        pointDeltas.x = Range.clip(pointDeltas.x, -moveSpeed, moveSpeed)
        pointDeltas.y = Range.clip(pointDeltas.y, -moveSpeed, moveSpeed)
        dh = Range.clip(dh, -moveSpeed, moveSpeed)

        val (x, y) = target.p.dbNormalize
        azusa.azuTelemetry.fieldOverlay().fillCircle(x, y, 3.0)

        azusa.driveTrain.powers = Pose(pointDeltas, Angle(dh, Angle.Unit.RAW))
    }

    private fun getDeltaH(curr: Pose, target: PathPoint): Double {
        return if (target is LockedPathPoint) {
            (target.h - curr.h).wrap().raw
        } else {
            val forward = (target.p - curr.p).atan2
            val back = forward + Angle(PI, Angle.Unit.RAD)
            val angleToForward = (forward - curr.h).wrap()
            val angleToBack = (back - curr.h).wrap()
            val autoAngle = if (angleToForward.abs < angleToBack.abs) forward else back
            (autoAngle - curr.h).wrap().raw
        }
    }

    fun followPath(azusa: Azusa, start: PathPoint, end: PathPoint) {
        val clip: Point = clipIntersection(start.p, end.p, azusa.currPose.p)
        val (x, y) = circleLineIntersection(clip, start.p, end.p, end.followDistance)
        val followPoint = end.copy
        followPoint.p.x = x
        followPoint.p.y = y
        azusa.azuTelemetry.fieldOverlay()
            .setFill("white")
            .fillCircle(followPoint.p.dbNormalize.x, followPoint.p.dbNormalize.y, 2.0)
        goToPosition(azusa, followPoint)
    }
}
