package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.control.controllers.PurePursuitController
import org.firstinspires.ftc.teamcode.control.path.funcs.Functions
import org.firstinspires.ftc.teamcode.control.path.waypoints.PointTurnWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.StopWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import org.firstinspires.ftc.teamcode.hardware.Azusa
import kotlin.collections.ArrayList
import kotlin.math.absoluteValue

class Path(
    var waypoints: ArrayList<Waypoint>,
    val maxSpeed: Double
) {
    private var currWaypoint: Int
    private var interrupting: Boolean

    fun follow(azusa: Azusa) {
        val currPose = azusa.currPose

        if (interrupting) {
            val advance = (waypoints[currWaypoint].func as Functions.InterruptFunction).run(azusa, this)
            if (advance)
                interrupting = false
            else return
        }

        var skip: Boolean
        do {
            skip = false
            val target = waypoints[currWaypoint + 1]

            if (target is StopWaypoint) {
                if (currPose.distance(target) < 0.8)
                    skip = true
            } else if (target is PointTurnWaypoint) {
                if ((currPose.h - target.h).rad < target.dh.rad)
                    skip = true
            } else {
                if (azusa.currPose.distance(target) < target.followDistance)
                    skip = true
            }

            var currAction = waypoints[currWaypoint].func
            if (currAction is Functions.RepeatFunction) {
                currAction.run(azusa, this)
            } else if (currAction is Functions.LoopUntilFunction) {
                skip = currAction.run(azusa, this)
            }

            if (skip) {
                currWaypoint++

                currAction = waypoints[currWaypoint].func
                if (currAction is Functions.SimpleFunction) {
                    currAction.run(azusa, this)
                }
                if (currAction is Functions.InterruptFunction) {
                    interrupting = true
                    this.follow(azusa)
                    return
                }
            }
        } while (skip && currWaypoint < waypoints.size - 1)
        if (finished()) return

        val target = waypoints[currWaypoint + 1]

        if (target is StopWaypoint /*&& azusa.currPose.distance(target) < target.followDistance*/) {
            PurePursuitController.goToPosition(azusa, target, maxSpeed)
        } else if (target is PointTurnWaypoint) {
            PurePursuitController.goToPosition(azusa, target, maxSpeed)
        } else {
            PurePursuitController.followPath(azusa, waypoints[currWaypoint], target, maxSpeed)
            val totalAbs = azusa.driveTrain.powers.x.absoluteValue + azusa.driveTrain.powers.y.absoluteValue
            if(totalAbs != 0.0) {
                azusa.driveTrain.powers.p /= totalAbs
            }
        }
    }

    fun finished() = currWaypoint >= waypoints.size - 1

    init {
        val newWaypoints = ArrayList<Waypoint>(waypoints.size)
        for (waypoint in waypoints) {
            newWaypoints.add(waypoint.copy)
        }
        waypoints = newWaypoints

        interrupting = false
        currWaypoint = 0
    }
}
