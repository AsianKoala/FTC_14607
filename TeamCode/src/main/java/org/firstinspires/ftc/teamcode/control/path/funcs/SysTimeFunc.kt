package org.firstinspires.ftc.teamcode.control.path.funcs

class SysTimeFunc(dt: Long, func: Functions.SimpleFunction) : SysFunc(func) {
    var targetTime: Long = dt + System.currentTimeMillis()
}
