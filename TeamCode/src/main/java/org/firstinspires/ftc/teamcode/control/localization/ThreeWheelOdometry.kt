package org.firstinspires.ftc.teamcode.control.localization

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.util.AzusaTelemetry
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.Pose
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin

@Config
class ThreeWheelOdometry(val startPose: Pose, val startL: Int, val startR: Int, val startA: Int) {
    companion object {
        const val TICKS_PER_INCH = 1103.8839
        @JvmField var turnScalar: Double = 14.9691931
        @JvmField var auxTracker: Double = 3.85
    }

    private var currentPosition: Pose = startPose.copy

    var lastLeftEncoder = 0
    var lastRightEncoder = 0
    var lastAuxEncoder = 0

    var accumHeading = 0.0

    fun update(azuTelemetry: AzusaTelemetry, currLeftEncoder: Int, currRightEncoder: Int, currAuxEncoder: Int): Pose {
        azuTelemetry.addData("turn scalar", turnScalar)
        azuTelemetry.addData("aux tracker", auxTracker)

        val actualCurrLeft = -(currLeftEncoder - startL)
        val actualCurrRight = (currRightEncoder - startR)
        val actualCurrAux = (currAuxEncoder - startA)

        val lWheelDelta = (actualCurrLeft - lastLeftEncoder) / TICKS_PER_INCH
        val rWheelDelta = (actualCurrRight - lastRightEncoder) / TICKS_PER_INCH
        val aWheelDelta = (actualCurrAux - lastAuxEncoder) / TICKS_PER_INCH

        val leftTotal = actualCurrLeft / TICKS_PER_INCH
        val rightTotal = actualCurrRight / TICKS_PER_INCH

        val lastAngle = currentPosition.h.copy
        currentPosition.h = -Angle(((leftTotal - rightTotal) / turnScalar), AngleUnit.RAD) + startPose.h

        val angleIncrement = (lWheelDelta - rWheelDelta) / turnScalar
        val auxPrediction = angleIncrement * auxTracker
        val rX = aWheelDelta - auxPrediction

        accumHeading += angleIncrement
        azuTelemetry.addData("calc turnScalar", (accumHeading / (2 * PI * 15)) * turnScalar)

        var deltaY = (lWheelDelta - rWheelDelta) / 2.0
        var deltaX = rX

        if (angleIncrement.absoluteValue > 0) {
            val radiusOfMovement = (lWheelDelta + rWheelDelta) / (2 * angleIncrement)
            val radiusOfStrafe = rX / angleIncrement

            deltaX = radiusOfMovement * (1 - cos(angleIncrement)) + (radiusOfStrafe * sin(angleIncrement))
            deltaY = (radiusOfMovement * sin(angleIncrement)) + (radiusOfStrafe * (1 - cos(angleIncrement)))
        }

        currentPosition.p.x += lastAngle.cos * deltaY - lastAngle.sin * deltaX
        currentPosition.p.y += lastAngle.sin * deltaY + lastAngle.cos * deltaX

        Speedometer.xDistTraveled += rX
        Speedometer.yDistTraveled += deltaY

        lastLeftEncoder = actualCurrLeft
        lastRightEncoder = actualCurrRight
        lastAuxEncoder = actualCurrAux

        return currentPosition
    }
}
