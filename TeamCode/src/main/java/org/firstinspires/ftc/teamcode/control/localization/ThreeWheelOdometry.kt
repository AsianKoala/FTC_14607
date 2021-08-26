package org.firstinspires.ftc.teamcode.control.localization

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.util.math.*
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

    private var allRawDeltas: ArrayList<TimePose> = ArrayList()
    private var currVelocity: Pose = Pose(Point.ORIGIN, Angle(0.0, AngleUnit.RAD))

    private var totalX = 0.0
    private var totalY = 0.0
    private var totalH = 0.0

    var lastLeftEncoder = 0
    var lastRightEncoder = 0
    var lastAuxEncoder = 0

    var accumHeading = 0.0

    fun update(currLeftEncoder: Int, currRightEncoder: Int, currAuxEncoder: Int): Pose {

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

        // calc vel
//        allRawDeltas.add(TimePose(Pose(Point(deltaX, deltaY), Angle(angleIncrement, AngleUnit.RAD))))
//        val bestIndex:  = if(allRawDeltas.size < 6) 0 else allRawDeltas.size - 6
//        val currRawDelta = allRawDeltas[allRawDeltas.size - 1]

        lastLeftEncoder = actualCurrLeft
        lastRightEncoder = actualCurrRight
        lastAuxEncoder = actualCurrAux

        return currentPosition
    }
}
