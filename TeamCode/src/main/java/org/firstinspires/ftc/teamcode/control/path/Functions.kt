package org.firstinspires.ftc.teamcode.control.path

import org.firstinspires.ftc.teamcode.control.system.Azusa
import org.firstinspires.ftc.teamcode.util.Pose

class Functions {
    interface Function
    interface SimpleFunction : Function {
        fun run(azusa: Azusa, path: Path)
    }

    interface LoopUntilFunction : Function {
        fun run(azusa: Azusa, path: Path): Boolean
    }

    // these give higher priority to path
    interface RepeatFunction : Function {
        fun run(azusa: Azusa, path: Path)
    }

    // highest prio
    interface InterruptFunction : Function {
        fun run(azusa: Azusa, path: Path): Boolean
    }

    class TimeFunction(dt: Long, var func: Function) {
        var targetTime: Long = dt + System.currentTimeMillis()
    }

    class PoseFunction(var targetPose: Pose, var func: Function)
}
