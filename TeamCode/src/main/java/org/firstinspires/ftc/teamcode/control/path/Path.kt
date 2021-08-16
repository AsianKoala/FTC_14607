package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.control.controllers.PurePursuitController
import org.firstinspires.ftc.teamcode.hardware.Azusa
import java.util.LinkedList
import kotlin.collections.ArrayList

class Path(
    val path: LinkedList<PathPoint>,
    val name: String = "default"
) : LinkedList<PathPoint>() {
    lateinit var curr: PathPoint
    var initialPoints: ArrayList<PathPoint> = ArrayList()
    var interrupting = false

    private var firstFinish = false

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
        val currPose = azusa.currPose
        var skip: Boolean

        if (interrupting) {
            val advance = (curr.func as Functions.InterruptFunction).run(azusa, this)
            if (advance)
                interrupting = false
            else return
        }

        do {
            skip = when (target) {
                is StopPathPoint -> currPose.distance(target) < 1
                is OnlyTurnPoint -> (currPose.h - target.h).rad < target.dh.rad
                else -> azusa.currPose.distance(target) < target.followDistance
            }

            var currAction = curr.func
            if (currAction is Functions.RepeatFunction) {
                currAction.run(azusa, this)
            } else if (currAction is Functions.LoopUntilFunction) {
                skip = currAction.run(azusa, this)
            }

            if (skip) {
                curr = target.copy
                removeFirst()

                currAction = curr.func
                if (currAction is Functions.SimpleFunction) {
                    currAction.run(azusa, this)
                }
                if (currAction is Functions.InterruptFunction) {
                    interrupting = true
                    this.follow(azusa)
                    return
                }
            }
        } while (skip && !isEmpty())
        if (isEmpty()) return

        if (target is StopPathPoint && azusa.currPose.distance(target) < target.followDistance) {
            PurePursuitController.goToPosition(azusa, target)
        } else if(target is OnlyTurnPoint) {
            PurePursuitController.goToPosition(azusa, target)
        } else {
            PurePursuitController.followPath(azusa, curr, target)
        }
    }

    override fun toString(): String {
        return name
    }
}
