package org.firstinspires.ftc.teamcode.control.localization

import org.firstinspires.ftc.teamcode.control.test.KalmanFilter
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.Pose
import org.firstinspires.ftc.teamcode.util.opmode.AzusaTelemetry

class SuperOdo(private val startPose: Pose, private val startL: Int, private val startR: Int, private val startA: Int) {
    companion object {
        const val TICKS_PER_INCH = 1103.8839
        @JvmField var turnScalar: Double = 14.9691931
        @JvmField var perpScalar: Double = 3.85

        @JvmField var Q = 0.1
        @JvmField var R = 0.4
        @JvmField var P = 1.0
        @JvmField var K = 1.0
    }

    val kalmanFilter = KalmanFilter(startPose.h.angle, Q, R, P, K)

    private var currentPosition: Pose = startPose.copy

    var lastLeftEncoder = 0
    var lastRightEncoder = 0
    var lastPerpEncoder = 0

    var mintime = 0.0
    var lastIMURead: Long = 0
    var lastIMUAngle = 0.0
    var imuBias = Angle.EAST

    fun update(azuTelemetry: AzusaTelemetry, currLeftEncoder: Int, currRightEncoder: Int, currPerpEncoder: Int, imuHeading: Double): Pose {

        val actualCurrLeft = -(currLeftEncoder - startL)
        val actualCurrRight = (currRightEncoder - startR)
        val actualCurrPerp = (currPerpEncoder - startA)

        val lWheelDelta = (actualCurrLeft - lastLeftEncoder) / TICKS_PER_INCH
        val rWheelDelta = (actualCurrRight - lastRightEncoder) / TICKS_PER_INCH
        val aWheelDelta = (actualCurrPerp - lastPerpEncoder) / TICKS_PER_INCH

        val leftTotal = actualCurrLeft / TICKS_PER_INCH
        val rightTotal = actualCurrRight / TICKS_PER_INCH

        val lastAngle = currentPosition.h.copy

        val angleIncrement = (lWheelDelta - rWheelDelta) / turnScalar
        val odocalcangle = -Angle(((leftTotal - rightTotal) / turnScalar), AngleUnit.RAD) + startPose.h + imuBias

        if (System.currentTimeMillis() - mintime > lastIMURead) {
            lastIMUAngle = imuHeading
            lastIMURead = System.currentTimeMillis()

            imuBias = Angle(lastIMUAngle, AngleUnit.RAD) - odocalcangle
        } else {
            lastIMUAngle += angleIncrement
        }

        val finalAngle = kalmanFilter.update(odocalcangle.angle, lastIMUAngle)
        azuTelemetry.addData("finalAngle: ", finalAngle)

        val perpPredict = angleIncrement * perpScalar
        val dx = aWheelDelta - perpPredict

        val (deltaX, deltaY) = EulerIntegration.update(dx, lWheelDelta, rWheelDelta, angleIncrement)

        currentPosition.p.x += lastAngle.cos * deltaY - lastAngle.sin * deltaX
        currentPosition.p.y += lastAngle.sin * deltaY + lastAngle.cos * deltaX

        lastLeftEncoder = actualCurrLeft
        lastRightEncoder = actualCurrRight
        lastPerpEncoder = actualCurrPerp

        return currentPosition
    }
}
