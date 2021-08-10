package org.firstinspires.ftc.teamcode.control.localization

import org.firstinspires.ftc.teamcode.control.system.Azusa
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose

class ThreeWheelOdometry(start: Pose) {
    companion object {
        const val TICKS_PER_INCH = 1103.8839
        val trackerCoeffs = Point(8272.5 / DriftOdo.TICKS_PER_INCH, -8651 / DriftOdo.TICKS_PER_INCH) // 8672 -7158
        const val PERP_PORT = 0
        const val LEFT_PORT = 1
        const val RIGHT_PORT = 2
    }

    private var startHeading: Angle = start.h
    private val currentPosition = Pose()

    private var currWheels = Pose()
    private var prevWheels = Pose(Point(), Angle(Angle.Unit.RAW))
    private var prevHeading: Angle = startHeading

    fun update(azusa: Azusa, leftScaled: Double, rightScaled: Double, perpScaled: Double): Pose {

        return currentPosition
    }
}
