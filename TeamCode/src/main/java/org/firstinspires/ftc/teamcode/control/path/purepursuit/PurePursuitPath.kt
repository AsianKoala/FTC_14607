package org.firstinspires.ftc.teamcode.control.path.purepursuit

import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.PurePursuitController
import org.firstinspires.ftc.teamcode.control.path.funcs.Functions
import org.firstinspires.ftc.teamcode.control.path.waypoints.PointTurnWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.StopWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import org.firstinspires.ftc.teamcode.hardware.Azusa
import org.firstinspires.ftc.teamcode.util.math.MathUtil
import org.firstinspires.ftc.teamcode.util.math.Point

class PurePursuitPath(waypoints: ArrayList<Waypoint>) : Path(waypoints) {
    override fun update(azusa: Azusa) {
        val currPose = azusa.currPose

        val target = target

        var skip: Boolean
        do {
            skip = when (target) {
                is StopWaypoint -> currPose.distance(target) < 0.8 && MathUtil.angleThresh(currPose.h, target.h, target.dh)
                is PointTurnWaypoint -> MathUtil.angleThresh(currPose.h, target.h, target.dh)
                else -> currPose.distance(target) < target.followDistance
            }
5
            val startAction = start.func
            if (startAction is Functions.RepeatFunction) {
                startAction.run(azusa, this)
            } else if (startAction is Functions.LoopUntilFunction) {
                skip = startAction.run(azusa, this)
            }

            if (skip) {
                println(incWaypoint())

                val currAction = start.func
                if (currAction is Functions.SimpleFunction) {
                    currAction.run(azusa, this)
                }
            }
        } while (skip && !this.isFinished)

        val nStart = start
        val nEnd = target

        val clip: Point = MathUtil.clipIntersection(nStart.p, nEnd.p, azusa.currPose.p)
        val (x, y) = MathUtil.circleLineIntersection(clip, nStart.p, nEnd.p, nEnd.followDistance)
        val followPoint = nEnd.copy
        followPoint.x = x
        followPoint.y = y

        azusa.azuTelemetry.addData("followpoint", followPoint.p)
        azusa.azuTelemetry.fieldOverlay()
            .setStroke("white")
            .strokeLine(azusa.currPose.p.dbNormalize.x, azusa.currPose.p.dbNormalize.y, followPoint.p.dbNormalize.x, followPoint.p.dbNormalize.y)

        if ((target is StopWaypoint && azusa.currPose.distance(target) < target.followDistance) || target is PointTurnWaypoint) {
            PurePursuitController.goToPosition(azusa, nEnd)
        } else {
            PurePursuitController.goToPosition(azusa, followPoint)
        }
    }
}
