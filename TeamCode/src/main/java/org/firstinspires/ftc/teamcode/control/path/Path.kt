package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import kotlin.collections.ArrayList

class Path(waypoints: ArrayList<Waypoint>) : ArrayList<Waypoint>() {

    var currWaypoint: Int = 0
        private set

    val start get() = this[currWaypoint]
    val target get() = this[currWaypoint + 1]

    val isFinished = currWaypoint >= size - 1

    fun incWaypoint() = ++currWaypoint

    init {
        waypoints.forEach { this.add(it.copy) }
    }
}
