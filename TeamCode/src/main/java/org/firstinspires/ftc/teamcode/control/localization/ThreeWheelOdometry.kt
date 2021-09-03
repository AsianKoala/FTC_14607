package org.firstinspires.ftc.teamcode.control.localization

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.util.math.*
import org.firstinspires.ftc.teamcode.util.opmode.AzusaTelemetry

@Config
class ThreeWheelOdometry(private val startPose: Pose, private val startL: Int, private val startR: Int, private val startA: Int) {
    companion object {
        const val TICKS_PER_INCH = 1103.8839
        @JvmField var turnScalar: Double = 14.9691931
        @JvmField var perpScalar: Double = 3.85
    }

    private var currentPosition: Pose = startPose.copy

    var lastLeftEncoder = 0
    var lastRightEncoder = 0
    var lastPerpEncoder = 0

    var accumHeading = 0.0

    fun update(azuTelemetry: AzusaTelemetry, currLeftEncoder: Int, currRightEncoder: Int, currPerpEncoder: Int): Pose {

        val actualCurrLeft = -(currLeftEncoder - startL)
        val actualCurrRight = (currRightEncoder - startR)
        val actualCurrPerp = (currPerpEncoder - startA)

        val lWheelDelta = (actualCurrLeft - lastLeftEncoder) / TICKS_PER_INCH
        val rWheelDelta = (actualCurrRight - lastRightEncoder) / TICKS_PER_INCH
        val aWheelDelta = (actualCurrPerp - lastPerpEncoder) / TICKS_PER_INCH

        val leftTotal = actualCurrLeft / TICKS_PER_INCH
        val rightTotal = actualCurrRight / TICKS_PER_INCH

        val lastAngle = currentPosition.h.copy
        currentPosition.h = -Angle(((leftTotal - rightTotal) / turnScalar), AngleUnit.RAD) + startPose.h

        val angleIncrement = (lWheelDelta - rWheelDelta) / turnScalar
        val perpPredict = angleIncrement * perpScalar
        val dx = aWheelDelta - perpPredict

//        accumHeading += angleIncrement
//        azuTelemetry.addData("calc turnScalar", (accumHeading / (2 * PI * 15)) * turnScalar)

        val (deltaX, deltaY) = EulerIntegration.update(dx, lWheelDelta, rWheelDelta, angleIncrement)

        currentPosition.p.x += lastAngle.cos * deltaY - lastAngle.sin * deltaX
        currentPosition.p.y += lastAngle.sin * deltaY + lastAngle.cos * deltaX

        lastLeftEncoder = actualCurrLeft
        lastRightEncoder = actualCurrRight
        lastPerpEncoder = actualCurrPerp

        return currentPosition
    }
}
