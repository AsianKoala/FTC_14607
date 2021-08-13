package org.firstinspires.ftc.teamcode.control.localization

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.Pose

@Config
class ThreeWheelOdometry(val startPose: Pose) {
    companion object {
        const val TICKS_PER_INCH = 1103.8839
    }

    val turnScalar = 8272.5 / DriftOdo.TICKS_PER_INCH
    val xTracker = -8651 / DriftOdo.TICKS_PER_INCH

    var totalYTraveled = 0.0
    var totalXTraveled = 0.0
    var totalHTraveled = 0.0

    var lastLeftEncoder = 0
    var lastRightEncoder = 0
    var lastAuxEncoder = 0

    private var lastRawAngle = 0.0

    private var currentPosition: Pose = startPose

    fun update(currLeftEncoder: Int, currRightEncoder: Int, currAuxEncoder: Int): Pose {
        val leftDelta = (currLeftEncoder - lastLeftEncoder) / TICKS_PER_INCH
        val rightDelta = (currRightEncoder - lastRightEncoder) / TICKS_PER_INCH
        val auxDelta = (currAuxEncoder - lastAuxEncoder) / TICKS_PER_INCH

        val angleIncrement = (leftDelta - rightDelta) / turnScalar

        val leftTotal = currLeftEncoder / TICKS_PER_INCH
        val rightTotal = currRightEncoder / TICKS_PER_INCH
        lastRawAngle = ((leftTotal - rightTotal) / turnScalar)
        val finalAngle = lastRawAngle + startPose.h.rad

        val auxPrediction = angleIncrement * xTracker

        val yDelta = (leftDelta - rightDelta) / 2.0
        val xDelta = auxDelta - auxPrediction

        val data = ArcLocalizer.update(currentPosition, Pose(xDelta, yDelta, angleIncrement), Angle(finalAngle).wrap())

        totalXTraveled += data.deltaXVec
        totalYTraveled += data.deltaYVec
        totalHTraveled += angleIncrement

        lastLeftEncoder = currLeftEncoder
        lastRightEncoder = currRightEncoder
        lastAuxEncoder = currAuxEncoder

        return currentPosition
    }
}
