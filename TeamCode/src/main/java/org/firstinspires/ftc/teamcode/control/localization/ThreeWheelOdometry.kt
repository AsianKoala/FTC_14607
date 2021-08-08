package org.firstinspires.ftc.teamcode.control.localization

import org.firstinspires.ftc.teamcode.control.system.Azusa
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose
import org.openftc.revextensions2.RevBulkData

class ThreeWheelOdometry(start: Pose) {
    companion object {
        const val TICKS_PER_INCH = 1103.8839
        val trackerCoeffs = Point(8272.5 / DriftOdo.TICKS_PER_INCH, -8651 / DriftOdo.TICKS_PER_INCH) // 8672 -7158
        const val HORIZ_PORT = 0
        const val LEFT = 1
        const val RIGHT = 2
    }

    private var startHeading: Angle = start.h
    private val currentPosition = Pose()

    private var currWheels = Pose()
    private var prevWheels = Pose(Point(), Angle(Angle.Unit.RAW))
    private var prevHeading: Angle = startHeading

    fun update(azusa: Azusa, data: RevBulkData): Pose {
        currWheels = Pose(Point(
                data.getMotorCurrentPosition(LEFT).toDouble(),
                data.getMotorCurrentPosition(RIGHT).toDouble()),
                Angle(data.getMotorCurrentPosition(HORIZ_PORT).toDouble(), Angle.Unit.RAW)
        )
        val deltaLeftScaled = (currWheels.x - prevWheels.x) / TICKS_PER_INCH
        val deltaRightScaled = (currWheels.y - prevWheels.y) / TICKS_PER_INCH
        val deltaHorizScaled = ((currWheels.h - prevWheels.h) / TICKS_PER_INCH).raw

        val dTheta = Angle((deltaLeftScaled - deltaRightScaled) / trackerCoeffs.y, Angle.Unit.RAD)
        val h = Angle(((currWheels.x - currWheels.y) / trackerCoeffs.y) + prevHeading.rad).wrap()

//        currentPosition.p += Point(
//                -(h.cos * correctedDeltas.y) + h.sin * correctedDeltas.x,
//                -(h.sin * correctedDeltas.y) - h.cos * correctedDeltas.x
//        )
        currentPosition.h = h

        val trackerXScaled = trackerCoeffs.x / (Math.PI / 2)
        val trackerX = deltaHorizScaled - (trackerXScaled * dTheta.angle)


        return currentPosition
    }
}