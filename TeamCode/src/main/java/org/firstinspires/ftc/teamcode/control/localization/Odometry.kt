package org.firstinspires.ftc.teamcode.control.localization

import org.firstinspires.ftc.teamcode.util.Pose
import org.firstinspires.ftc.teamcode.control.localization.OdometrySet
import org.firstinspires.ftc.teamcode.control.localization.Odometry
import org.firstinspires.ftc.teamcode.util.Point

@Deprecated("fuck you")
class Odometry(start: Pose, odometrySet: OdometrySet) {
//    private var prevVertical: Int
//    private var prevHorizontal: Int
//    private var prevHeading: Double
//    private val odometrySet: OdometrySet
//    fun setStart(start: Pose) {
//        startHeading = start.h
//        prevHeading = startHeading
//        currentPosition = start
//    }
//
//    // very very dangerous
//    fun setGlobalPosition(newPosition: Point?) {
//        currentPosition = Pose(newPosition!!, currentPosition.h)
//    }
//
//    fun update(heading: Double) {
//        val deltaY = (odometrySet.verticalTicks - prevVertical) / TICKS_PER_INCH
//        val deltaX = (odometrySet.horizontalTicks - prevHorizontal) / TICKS_PER_INCH
//        val deltaAngle: Double = MathUtil.angleWrap(heading - prevHeading)
//
////        double newHeading = MathUtil.angleWrap(currentPosition.h + deltaAngle);
//        currentPosition.x += -(Math.cos(heading) * deltaY) + Math.sin(heading) * deltaX
//        currentPosition.y += -(Math.sin(heading) * deltaY) - Math.cos(heading) * deltaX
//        currentPosition.h = heading + startHeading
//        prevHorizontal = odometrySet.horizontalTicks
//        prevVertical = odometrySet.verticalTicks
//        prevHeading = currentPosition.h
//    }
//
//    override fun toString(): String {
//        return "v: " + odometrySet.verticalTicks + " , " + "h: " + odometrySet.horizontalTicks
//    }
//
//    companion object {
//        // 8192 ticks per revolution
//        // wheels are 60mm, or 2.3622 inches diameter
//        // 2.3622 * pi = 7.42107016631 circumference
//        // 8192 / 7.42107016631 = ticks per inch
//        const val TICKS_PER_INCH = 1103.8839
//        var startHeading: Double
//        var currentPosition: Pose
//    }
//
//    init {
//        startHeading = start.h
//        prevHorizontal = 0
//        prevVertical = 0
//        prevHeading = startHeading
//        this.odometrySet = odometrySet
//        currentPosition = start
//    }
}