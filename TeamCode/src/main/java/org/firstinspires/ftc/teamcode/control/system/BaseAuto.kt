package org.firstinspires.ftc.teamcode.control.system

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.PurePursuitController
import java.util.*

@Disabled
abstract class BaseAuto : BaseOpMode() {
    abstract fun initialPath(): Path

    lateinit var pathCache: Path

    lateinit var x: DoubleArray
    lateinit var y: DoubleArray

    override fun onInit() {
        pathCache = initialPath()

        x = DoubleArray(pathCache.size)
        y = DoubleArray(pathCache.size)
        for ((index, e) in pathCache.withIndex()) {
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

        PurePursuitController.followPath(azusa, pathCache)
        updateDashboardPath()
    }

    private fun updateDashboardPath() {
        azusaTelemetry.fieldOverlay()
            .setStroke("black")
            .strokePolyline(x, y)
    }
}
