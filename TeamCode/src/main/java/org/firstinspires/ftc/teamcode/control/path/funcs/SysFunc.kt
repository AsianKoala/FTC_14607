package org.firstinspires.ftc.teamcode.control.path.funcs

class SysFunc(var time: Double, val func: Func) {

    init {
        time += System.currentTimeMillis()
    }
}
