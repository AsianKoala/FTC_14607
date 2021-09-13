package org.firstinspires.ftc.teamcode.control.system

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.PurePursuitController
import org.firstinspires.ftc.teamcode.control.path.purepursuit.PurePursuitPath
import org.firstinspires.ftc.teamcode.util.opmode.Globals
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
        updateDashboardPath()
    }

    override fun onLoop() {
        if (pathCache.isFinished) {
            azusa.driveTrain.setZeroPowers()
            requestOpModeStop()
        }

        pathCache.update(azusa)
        updateDashboardPath()
    }

    override fun onStop() {
        Globals.AUTO_END_POSE = azusa.currPose.copy
    }

    private fun updateDashboardPath() {
        azusaTelemetry.fieldOverlay()
            .setStroke("black")
            .strokePolyline(x, y)
    }
}
