package org.firstinspires.ftc.teamcode.control.path.builders

import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.PathPoint
import java.util.*

class PathBuilder(var name: String) {
    var path : LinkedList<PathPoint> = LinkedList<PathPoint>()
    fun addPoint(p: PathPoint): PathBuilder {
        path.add(p)
        return this
    }

    fun build(): Path = Path(path, name)
}