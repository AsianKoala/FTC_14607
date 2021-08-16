package org.firstinspires.ftc.teamcode.control.localization

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.Pose
import kotlin.math.cos
import kotlin.math.sin

@Config
class ThreeWheelOdometry(val startPose: Pose, val startL: Int, val startR: Int, val startA: Int) {
    companion object {
        const val TICKS_PER_INCH = 1103.8839
    }

    val turnScalar: Double = 16.0
    val xTracker: Double = 4.0
    
    var lastLeftEncoder = 0
    var lastRightEncoder = 0
    var lastAuxEncoder = 0

    private var currentPosition: Pose = startPose.copy

    fun update(telemetry: Telemetry, currLeftEncoder: Int, currRightEncoder: Int, currAuxEncoder: Int): Pose {
        val actualCurrLeft = currLeftEncoder - startL
        val actualCurrRight = -(currRightEncoder - startR)
        val actualCurrAux = currAuxEncoder - startA

        telemetry.addData("left wheel", actualCurrLeft)
        telemetry.addData("right wheel", actualCurrRight)
        telemetry.addData("aux wheel", actualCurrAux)

        val lWheelDelta = (actualCurrLeft - lastLeftEncoder) / TICKS_PER_INCH
        val rWheelDelta = (actualCurrRight - lastRightEncoder) / TICKS_PER_INCH
        val aWheelDelta = (actualCurrAux - lastAuxEncoder) / TICKS_PER_INCH

        telemetry.addData("left delta", lWheelDelta)
        telemetry.addData("right delta", rWheelDelta)
        telemetry.addData("aux delta", aWheelDelta)

        val angleIncrement = (lWheelDelta - rWheelDelta) / turnScalar
        telemetry.addData("angle increment", angleIncrement)

        val leftTotal = actualCurrLeft / TICKS_PER_INCH
        val rightTotal = actualCurrRight / TICKS_PER_INCH

        telemetry.addData("left total", leftTotal)
        telemetry.addData("right total", rightTotal)

        val lastAngle = currentPosition.h.copy
        currentPosition.h = Angle(((leftTotal + rightTotal) / turnScalar), Angle.Unit.RAD) + startPose.h

        telemetry.addData("last angle", lastAngle)
        telemetry.addData("left total", leftTotal)
        telemetry.addData("right total", rightTotal)

        telemetry.addData("startpose h", startPose.h)

        val auxPrediction = angleIncrement * xTracker

        val rX = aWheelDelta - auxPrediction

        var deltaY = (lWheelDelta - rWheelDelta) / 2.0
        var deltaX = rX

//        if (angleIncrement.absoluteValue > 0) {
//            val radiusOfMovement = (leftDelta + rightDelta) / (2 * angleIncrement)
//            val radiusOfStrafe = auxPrediction / angleIncrement
//
//            deltaX = radiusOfMovement * (1 - cos(angleIncrement)) + (radiusOfStrafe * sin(angleIncrement))
//            deltaY = (radiusOfMovement * sin(angleIncrement)) + (radiusOfStrafe * (1 - cos(angleIncrement)))
//        }

        telemetry.addData("predicted rX", rX)
        telemetry.addData("deltaY", deltaY)

        currentPosition.p.x += lastAngle.cos * deltaY + lastAngle.sin * deltaX
        currentPosition.p.y += lastAngle.sin * deltaY - lastAngle.cos * deltaX

        Speedometer.xDistTraveled += rX
        Speedometer.yDistTraveled += deltaY

        lastLeftEncoder = actualCurrLeft
        lastRightEncoder = actualCurrRight
        lastAuxEncoder = actualCurrAux

        return currentPosition
    }
}
