package org.firstinspires.ftc.teamcode.control.system

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.util.math.Point
import java.util.*

@Disabled
abstract class BaseAuto : BaseOpMode() {
    abstract fun initialPath(): Path

    lateinit var pathCache: Path

    lateinit var x: DoubleArray
    lateinit var y: DoubleArray

    private val lastPositions = LinkedList<Point>()
    var lastUpdateTime: Long = 0

    override fun onInit() {
        pathCache = initialPath()

        x = DoubleArray(pathCache.waypoints.size)
        y = DoubleArray(pathCache.waypoints.size)
        for ((index, e) in pathCache.waypoints.withIndex()) {
            x[index] = e.p.y
            y[index] = -e.p.x
        }
    }

    override fun onInitLoop() {
        azusaTelemetry.fieldOverlay()
                .setStroke("black")
                .strokePolyline(x, y)
    }
    override fun onLoop() {
        if (pathCache.finished()) {
            azusa.driveTrain.setZeroPowers()
            requestOpModeStop()
        }

        pathCache.follow(azusa)

        azusaTelemetry.fieldOverlay()
                .setStroke("black")
                .strokePolyline(x, y)

        if(System.currentTimeMillis() - lastUpdateTime > 250) {
            lastUpdateTime = System.currentTimeMillis()
            lastPositions.add(azusa.currPose.p.copy)
        }

        val iterator = lastPositions.iterator()
        while(iterator.hasNext()) {
            val curr = iterator.next()
            azusaTelemetry.fieldOverlay()
                    .setStroke("red")
                    .strokeCircle(curr.dbNormalize.x, curr.dbNormalize.y, 1.0)
        }
    }
}
