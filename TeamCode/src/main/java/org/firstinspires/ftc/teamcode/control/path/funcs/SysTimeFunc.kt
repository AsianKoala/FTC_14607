package org.firstinspires.ftc.teamcode.control.path.funcs

class SysTimeFunc(dt: Long, var func: Functions.Function) : SysFunc() {
    var targetTime: Long = dt + System.currentTimeMillis()
}
