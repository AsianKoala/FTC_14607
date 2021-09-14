package org.firstinspires.ftc.teamcode.control.path.funcs

import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.hardware.Azusa

fun interface InterruptFunction : Functions.Function {
    fun run(azusa: Azusa, path: Path): Boolean
}