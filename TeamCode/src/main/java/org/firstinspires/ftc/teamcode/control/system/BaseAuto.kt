package org.firstinspires.ftc.teamcode.control.system

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.control.path.Path

// this is where we do all of our pp logic
@Disabled
abstract class BaseAuto : BaseOpMode() {
    abstract fun initialPath(): Path

    lateinit var pathCache: Path

    override fun onInit() {
        pathCache = initialPath()
    }
    
    override fun onLoop() {
        if (!pathCache.isEmpty()) {
            pathCache.follow(robot)
        } else {
            robot.driveTrain.setZeroPowers()
            requestOpModeStop()
        }
    }
}
