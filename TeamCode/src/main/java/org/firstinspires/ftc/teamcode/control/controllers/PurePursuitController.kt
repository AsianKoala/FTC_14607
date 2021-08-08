package org.firstinspires.ftc.teamcode.control.controllers

import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.control.path.LockedPathPoint
import org.firstinspires.ftc.teamcode.control.path.PathPoint
import org.firstinspires.ftc.teamcode.control.system.Azusa
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.MathUtil.circleLineIntersection
import org.firstinspires.ftc.teamcode.util.MathUtil.clipIntersection
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose
import kotlin.math.PI
import kotlin.math.absoluteValue

object PurePursuitController {
//    private val mins = Pose(0.11, 0.09, 0.11)

    private fun relVals(curr: Pose, target: PathPoint): Pose {
        val d = (curr.p - target.p).hypot
        val rh = (target.p - curr.p).atan2 - curr.h
        return Pose(Point(-d * rh.sin, d * rh.cos), rh)
    }

    fun goToPosition(azusa: Azusa, target: PathPoint, start: PathPoint) {
        val relVals: Pose = relVals(azusa.currPose, target)
        azusa.packet.addData("relX", relVals.x)
        azusa.packet.addData("relY", relVals.y)

        val v = relVals.x.absoluteValue + relVals.y.absoluteValue
        val smoothing = 12.0 * v
        val powerPose = Pose(
            Point(
                relVals.x * relVals.x.absoluteValue,
                relVals.y * relVals.y.absoluteValue
            ),
            Angle(0.0, Angle.Unit.RAW)
        )

        powerPose.p /= smoothing

        val relAngle = getDesiredAngle(azusa.currPose, target)
        azusa.packet.addData("relH", relAngle.deg)
        powerPose.h = relAngle
        if (target is LockedPathPoint) {
            powerPose.h = Angle((target.h - azusa.currPose.h).rad / Angle(35.0, Angle.Unit.DEG).rad)
        }

        var turning = true
        if (relVals.hypot < 3) {
            powerPose.h.angle = 0.0
            turning = false
        }

        powerPose.p.x *= Range.clip(relVals.x / 3.0, 0.0, 1.0)
        powerPose.p.y *= Range.clip(relVals.y / 3.0, 0.0, 1.0)
        powerPose.h.times(Range.clip(powerPose.h.abs / Math.toRadians(5.0), 0.0, 1.0))

        val turnErrorScaler: Double = if (turning) Range.clip(
            1.0 - (relAngle.rad / Angle(40.0, Angle.Unit.DEG).rad).absoluteValue,
            0.4,
            0.8
        ) else 1.0

        azusa.packet.addData("turnErrorScalar", turnErrorScaler)
        powerPose.p.x *= turnErrorScaler
        powerPose.p.y *= turnErrorScaler

//        powerPose.x *= 1 - Range.clip(Math.abs(powerPose.h),0,0.8);
//        powerPose.y *= 1 - Range.clip(Math.abs(powerPose.h),0,0.8);
        azusa.driveTrain.powers = powerPose.clipAbs(1.0)
    }

    private fun getDesiredAngle(curr: Pose, target: PathPoint): Angle {
        val forward = (target.p - curr.p).atan2
        val back = forward + Angle(PI)
        val angleToForward = (forward - curr.h).wrap()
        val angleToBack = (back - curr.h).wrap()
        val autoAngle = if (angleToForward.abs < angleToBack.abs) forward else back
        return (autoAngle - curr.h).wrap()
    }

    fun followPath(azusa: Azusa, start: PathPoint, end: PathPoint) {
        val clip: Point = clipIntersection(start.p, end.p, azusa.currPose.p)
        val (x, y) = circleLineIntersection(clip, start.p, end.p, end.followDistance)
        val followPoint = end.copy
        followPoint.p.x = x
        followPoint.p.y = y
        azusa.packet.fieldOverlay()
            .setFill("white")
            .fillCircle(followPoint.p.dbNormalize.x, followPoint.p.dbNormalize.y, 2.0)
        goToPosition(azusa, followPoint, start)
    }
}
