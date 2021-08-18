package org.firstinspires.ftc.teamcode.control.localization.old

import org.firstinspires.ftc.teamcode.hardware.Azusa
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.Point
import org.firstinspires.ftc.teamcode.util.math.Pose
import org.openftc.revextensions2.RevBulkData

@Deprecated("in favor of ThreeWheelOdometry")
class DriftOdo(start: Pose) {
    companion object {
        const val TICKS_PER_INCH = 1103.8839
        val trackerCoeffs = Point(8272.5 / TICKS_PER_INCH, -8651 / TICKS_PER_INCH) // 8672 -7158
        const val HORIZ_PORT = 0
        const val VERT_PORT = 2
    }

    private var startHeading: Angle = start.h
    private val currentPosition: Pose

    private var currWheels: Point
    private var deltaScaled: Point
    private var trackerScaled: Point
    private var correctedDeltas: Point

    private var prevWheels: Point
    private var prevHeading: Angle

    fun update(azusa: Azusa, heading: Angle, data: RevBulkData): Pose {
        currWheels = Point(
            data.getMotorCurrentPosition(HORIZ_PORT).toDouble(),
            data.getMotorCurrentPosition(VERT_PORT).toDouble()
        )
        deltaScaled = (currWheels - prevWheels) / TICKS_PER_INCH
        val deltaAngle: Angle = (heading - prevHeading).wrap()

        trackerScaled = trackerCoeffs / (Math.PI / 2)
        correctedDeltas = deltaScaled - (trackerScaled * deltaAngle.angle)

        currentPosition.p += Point(
            -(heading.cos * correctedDeltas.y) + heading.sin * correctedDeltas.x,
            -(heading.sin * correctedDeltas.y) - heading.cos * correctedDeltas.x
        )
        currentPosition.h = (heading + startHeading).wrap()

        azusa.azuTelemetry.addData("curr", currWheels)
        azusa.azuTelemetry.addData("corrected", correctedDeltas)
        azusa.azuTelemetry.addData("delta H", deltaAngle)
        azusa.azuTelemetry.addData("prev wheels", prevWheels)

        prevWheels = currWheels
        prevHeading = currentPosition.h
        return currentPosition
    }

    init {
        prevWheels = Point.ORIGIN
        prevHeading = startHeading
        currentPosition = start
        currWheels = Point.ORIGIN
        deltaScaled = Point.ORIGIN
        trackerScaled = Point.ORIGIN
        correctedDeltas = Point.ORIGIN
    }
}
