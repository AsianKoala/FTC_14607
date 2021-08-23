package org.firstinspires.ftc.teamcode.control.path.builders

import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint

class PathBuilder() {
    var path: ArrayList<Waypoint> = ArrayList()
    fun addPoint(p: Waypoint): PathBuilder {
        path.add(p)
        return this
    }

    fun build(): Path = Path(path)
}
