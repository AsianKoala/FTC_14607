package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.util.Pose

abstract class Azusa() {
    abstract fun startPose(): Pose
    abstract fun path(): Path?

    lateinit var currPose: Pose
    lateinit var currVel: Pose

    var isPurePursuit = true
    var pathCache: Path? = null
        set(value) {
            if(value == null) {
                field = null
            } else {
                isPurePursuit = value.size != 1
                field = value
            }

        }
}
