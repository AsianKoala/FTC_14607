package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.control.controllers.PurePursuitController
import org.firstinspires.ftc.teamcode.control.system.Azusa
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.MathUtil.angleThresh
import java.util.LinkedList
import kotlin.collections.ArrayList

class Path(
    val path: LinkedList<PathPoint>,
    val name: String = "default"
) : LinkedList<PathPoint>() {
    lateinit var curr: PathPoint
    var initialPoints: ArrayList<PathPoint> = ArrayList()
    private var firstFinish: Boolean

    init {
        firstFinish = false
    }

    fun init() {
        for (pathPoint in path) {
            val copy = pathPoint.copy
            add(copy)
            initialPoints.add(copy)
        }
        curr = first.copy
        removeFirst()
        firstFinish = false
    }

    fun follow(azusa: Azusa) {
        val target = first
        if (true/*!isPurePursuit*/) {
            PurePursuitController.goToPosition(azusa, target) // TODO: FIX THIS SHIT
        } else {
            var skip: Boolean
            if (target is OnlyTurnPoint) {
                skip = angleThresh(azusa.currPose.h, target.h, Angle(0.5, Angle.Unit.DEG))
            } else if (target is StopPathPoint) {
                skip = azusa.currPose.distance(target) < 2
            } else {
                skip = azusa.currPose.distance(target) < target.followDistance
                if (size > 1 && get(1) is StopPathPoint) {
                    skip = azusa.currPose.distance(target) < 10
                }
            }

//            if(!target.functions.isEmpty()) {
//                target.functions.removeIf(f -> f.cond() && f.func());
//                skip = skip && target.functions.size() == 0;
//            } // TODO
            if (skip) {
                curr = target.copy // swap old target to curr start
                removeFirst()
            }
            if (isEmpty()) {
                firstFinish = true
                return
            }
            if (target is StopPathPoint && azusa.currPose.distance(target) < target.followDistance) {
                PurePursuitController.goToPosition(azusa, target)
            } else {
                PurePursuitController.followPath(azusa, curr, target)
            }
        }
    }

    override fun toString(): String {
        return name
    }

    fun finished(): Boolean {
        if (firstFinish) {
            firstFinish = false
            return true
        }
        return false
    }
}
