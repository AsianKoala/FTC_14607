package org.firstinspires.ftc.teamcode.util

data class CurvePoint(
    @JvmField var x: Double,
    @JvmField var y: Double,
    @JvmField var moveSpeed: Double,
    @JvmField var followDistance: Double,
    @JvmField var slowDownTurnRadians: Double,
    @JvmField var slowDownTurnAmount: Double,
    @JvmField var pointLength: Double
) {
    val toPoint = Point(x, y)
    fun setPoint(p: Point) {
        x = p.x
        y = p.y
    }
}
