package org.firstinspires.ftc.teamcode.control.path.funcs

import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.hardware.OldAzusa

fun interface InterruptFunction : Func {
    fun run(azusa: OldAzusa, path: Path): Boolean
}
