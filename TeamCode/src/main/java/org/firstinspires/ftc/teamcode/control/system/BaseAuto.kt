package org.firstinspires.ftc.teamcode.control.system

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.control.path.Path
import java.util.*

@Disabled
abstract class BaseAuto : BaseOpMode() {
    abstract fun initialPath(): Path

    lateinit var pathCache: Path

    lateinit var x: DoubleArray
    lateinit var y: DoubleArray

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
        pathCache.follow(azusa)
        if (pathCache.finished()) {
            azusa.driveTrain.setZeroPowers()
            requestOpModeStop()
        }

        azusaTelemetry.fieldOverlay()
            .setStroke("black")
            .strokePolyline(x, y)
    }
}
