package org.firstinspires.ftc.teamcode.control.localization

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.Pose
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin

@Config
class ThreeWheelOdometry(val startPose: Pose, val startL: Int, val startR: Int, val startA: Int) {
    companion object {
        const val TICKS_PER_INCH = 1103.8839
    }

    private var currentPosition: Pose = startPose.copy

    val turnScalar: Double = 15.17775
    val auxTracker: Double = 3.85

    var lastLeftEncoder = 0
    var lastRightEncoder = 0
    var lastAuxEncoder = 0

    var totalAuxTracker: Double = 0.0

    fun update(telemetry: Telemetry, currLeftEncoder: Int, currRightEncoder: Int, currAuxEncoder: Int, imuH: Double): Pose {
        val actualCurrLeft = -(currLeftEncoder - startL)
        val actualCurrRight = (currRightEncoder - startR)
        val actualCurrAux = (currAuxEncoder - startA)

        telemetry.addData("left wheel", actualCurrLeft)
        telemetry.addData("right wheel", actualCurrRight)
        telemetry.addData("aux wheel", actualCurrAux)

        val lWheelDelta = (actualCurrLeft - lastLeftEncoder) / TICKS_PER_INCH
        val rWheelDelta = (actualCurrRight - lastRightEncoder) / TICKS_PER_INCH
        val aWheelDelta = (actualCurrAux - lastAuxEncoder) / TICKS_PER_INCH

        telemetry.addData("left delta", lWheelDelta)
        telemetry.addData("right delta", rWheelDelta)
        telemetry.addData("aux delta", aWheelDelta)

        val leftTotal = actualCurrLeft / TICKS_PER_INCH
        val rightTotal = actualCurrRight / TICKS_PER_INCH

        telemetry.addData("left total", leftTotal)
        telemetry.addData("right total", rightTotal)

        val lastAngle = currentPosition.h.copy
        currentPosition.h = -Angle(((leftTotal - rightTotal) / turnScalar), Angle.Unit.RAD) + startPose.h

        val angleIncrement = (lWheelDelta - rWheelDelta) / turnScalar
        val auxPrediction = angleIncrement * auxTracker
        val rX = aWheelDelta - auxPrediction

        totalAuxTracker += auxPrediction
        telemetry.addData("total aux - tracker", actualCurrAux - totalAuxTracker * TICKS_PER_INCH)

        var deltaY = (lWheelDelta - rWheelDelta) / 2.0
        var deltaX = rX

        if (angleIncrement.absoluteValue > 0) {
            val radiusOfMovement = (lWheelDelta + rWheelDelta) / (2 * angleIncrement)
            val radiusOfStrafe = rX / angleIncrement

            deltaX = radiusOfMovement * (1 - cos(angleIncrement)) + (radiusOfStrafe * sin(angleIncrement))
            deltaY = (radiusOfMovement * sin(angleIncrement)) + (radiusOfStrafe * (1 - cos(angleIncrement)))
        }

        currentPosition.p.x += lastAngle.cos * deltaY + lastAngle.sin * deltaX
        currentPosition.p.y += lastAngle.sin * deltaY - lastAngle.cos * deltaX

        Speedometer.xDistTraveled += rX
        Speedometer.yDistTraveled += deltaY

        lastLeftEncoder = actualCurrLeft
        lastRightEncoder = actualCurrRight
        lastAuxEncoder = actualCurrAux

        telemetry.addData("encoder ticks per degree", currentPosition.h)

        return currentPosition
    }
}
