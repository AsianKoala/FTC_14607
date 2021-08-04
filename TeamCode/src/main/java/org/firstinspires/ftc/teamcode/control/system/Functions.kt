package org.firstinspires.ftc.teamcode.control.system

import org.firstinspires.ftc.teamcode.util.Pose

// typically we want everything to be a function so its easier for interrupts/listeners
class Functions {
    interface Function
    interface SimpleFunction : Function {
        fun run(azusa: Azusa?)
    }

    interface LoopFunction : Function {
        fun run(azusa: Azusa?): Boolean
    }

    interface InterruptFunction : Function {
        fun run(azusa: Azusa?): Boolean
    }

    class TimeFunction(dt: Long, var func: SimpleFunction) {
        var targetTime: Long = dt + System.currentTimeMillis()
    }

    class PoseFunction(var targetPose: Pose, var func: SimpleFunction)
}