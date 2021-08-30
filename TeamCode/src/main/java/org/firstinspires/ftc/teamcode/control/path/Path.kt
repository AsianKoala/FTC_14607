package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.control.path.funcs.Functions
import org.firstinspires.ftc.teamcode.control.path.waypoints.PointTurnWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.StopWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import org.firstinspires.ftc.teamcode.hardware.Azusa
import kotlin.collections.ArrayList
import kotlin.math.absoluteValue

class Path(
    var waypoints: ArrayList<Waypoint>,
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
            val target = waypoints[currWaypoint + 1]

            skip = when (target) {
                is StopWaypoint -> (currPose.distance(target) < 0.8 && (currPose.h - target.h).angle.absoluteValue < 2.0)
                is PointTurnWaypoint -> ((currPose.h - target.h).rad < target.dh.rad)
                else -> currPose.distance(target) < target.followDistance
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

        if (target is StopWaypoint && azusa.currPose.distance(target) < target.followDistance) {
            PurePursuitController.goToPosition(azusa, target)
        } else if (target is PointTurnWaypoint) {
            PurePursuitController.goToPosition(azusa, target)
        } else {
            PurePursuitController.followPath(azusa, waypoints[currWaypoint], target)
        }
    }

    fun finished() = currWaypoint >= waypoints.size - 1

    init {
        val copyList = ArrayList<Waypoint>()
        waypoints.forEach { copyList.add(it) }
        waypoints = copyList

        interrupting = false
        currWaypoint = 0
    }
}
