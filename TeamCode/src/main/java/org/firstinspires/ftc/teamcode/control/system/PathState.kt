package org.firstinspires.ftc.teamcode.control.system

import org.firstinspires.ftc.teamcode.control.path.Path

abstract class PathState : State() {
    abstract val path: Path
}
