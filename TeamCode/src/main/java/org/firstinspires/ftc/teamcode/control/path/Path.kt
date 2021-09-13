package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import org.firstinspires.ftc.teamcode.hardware.Azusa
import kotlin.collections.ArrayList

abstract class Path(val waypoints: ArrayList<Waypoint>) {
    var currWaypoint: Int = 0
        protected set

    val isFinished get() = currWaypoint >= waypoints.size - 1

    abstract fun update(azusa: Azusa)

    init {
        val copy = ArrayList<Waypoint>()
        waypoints.forEach { copy.add(it.copy) }
        waypoints.clear()
        waypoints.addAll(copy)
    }
}
