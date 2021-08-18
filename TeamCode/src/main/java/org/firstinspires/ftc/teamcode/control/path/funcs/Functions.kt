package org.firstinspires.ftc.teamcode.control.path.funcs

import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.hardware.Azusa

class Functions {
    interface Function

    interface SimpleFunction : Function {
        fun run(azusa: Azusa, path: Path)
    }

    // path prio
    interface RepeatFunction : Function {
        fun run(azusa: Azusa, path: Path)
    }

    // parallel
    interface LoopUntilFunction : Function {
        fun run(azusa: Azusa, path: Path): Boolean
    }

    // max prio
    interface InterruptFunction : Function {
        fun run(azusa: Azusa, path: Path): Boolean
    }
}
