package org.firstinspires.ftc.teamcode.control.localization

import kotlin.jvm.JvmOverloads
import org.firstinspires.ftc.teamcode.util.Pose
import org.apache.commons.math3.linear.DecompositionSolver
import org.openftc.revextensions2.RevBulkData
import org.firstinspires.ftc.teamcode.control.localization.TwoWheelTrackingLocalizer
import org.firstinspires.ftc.teamcode.control.system.Azusa
import org.apache.commons.math3.linear.RealMatrix
import org.apache.commons.math3.linear.MatrixUtils
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.firstinspires.ftc.teamcode.control.localization.EncoderWheel
import org.apache.commons.math3.linear.LUDecomposition
import org.firstinspires.ftc.teamcode.util.Point

@Deprecated("never coming back to this piece of shit")
class TwoWheelTrackingLocalizer {
//    var forwardSolver: DecompositionSolver
//    var prevWheelPositions: IntArray
//    var prevHeading = 0.0
//
//    // External interfaces
//    var poseUpdate: Pose
//        private set
//    private var relativeRobotMovement: Pose
//    fun update(data: RevBulkData, heading: Double) {
//        val deltas = doubleArrayOf(
//            encoderTicksToInches(
//                data.getMotorCurrentPosition(Azusa.PARALLEL_ENCODER_PORT) - prevWheelPositions[0],
//                PARALLEL_TICKS_PER_INCH
//            ),
//            encoderTicksToInches(
//                data.getMotorCurrentPosition(Azusa.PERP_ENCODER_PORT) - prevWheelPositions[1],
//                PERP_TICKS_PER_INCH
//            ),
//            MathUtil.angleWrap(heading - prevHeading)
//        )
//        prevWheelPositions[0] = data.getMotorCurrentPosition(Azusa.PARALLEL_ENCODER_PORT)
//        prevHeading = heading
//        prevWheelPositions[1] = data.getMotorCurrentPosition(Azusa.PERP_ENCODER_PORT)
//        updateFromRelative(deltas)
//    }
//
//    fun updateFromRelative(deltas: DoubleArray) {
//        val m = MatrixUtils.createRealMatrix(arrayOf(deltas))
//        val rawPoseDelta = forwardSolver.solve(m.transpose())
//        val robotPoseDelta = Pose(
//            rawPoseDelta.getEntry(0, 0),
//            rawPoseDelta.getEntry(1, 0),
//            rawPoseDelta.getEntry(2, 0)
//        )
//        relativeRobotMovement = relativeRobotMovement.add(robotPoseDelta)
//        poseUpdate = relativeOdometryUpdate(poseUpdate, robotPoseDelta)
//    }
//
//    private fun relativeOdometryUpdate(fieldPose: Pose, robotPoseDelta: Pose): Pose {
//        val dtheta: Double = robotPoseDelta.h
//        val sineTerm: Double
//        val cosTerm: Double
//        if (MathUtil.epsilonEquals(dtheta, 0)) {
//            sineTerm = 1.0 - dtheta * dtheta / 6.0
//            cosTerm = dtheta / 2.0
//        } else {
//            sineTerm = Math.sin(dtheta) / dtheta
//            cosTerm = (1 - Math.cos(dtheta)) / dtheta
//        }
//        val fieldPositionDelta = Point(
//            sineTerm * robotPoseDelta.x + cosTerm * robotPoseDelta.y,
//            cosTerm * robotPoseDelta.x + sineTerm * robotPoseDelta.y
//        )
//        val fieldPoseDelta = Pose(fieldPositionDelta.rotated(fieldPose.h), robotPoseDelta.h)
//        return fieldPose.add(fieldPoseDelta)
//    }
//
//    companion object {
//        var PARALLEL_TICKS_PER_INCH = 1103.8839
//        var PERP_TICKS_PER_INCH = 1103.8839
//        private const val PARALLEL_X = 6.75
//        private const val PARALLEL_Y = 4.25
//        private const val PERPENDICULAR_X = 6.75
//        private const val PERPENDICULAR_Y = -4.25
//        fun encoderTicksToInches(ticks: Int, ticksPerInch: Double): Double {
//            return ticks / ticksPerInch
//        }
//
//        fun inchesToEncoderTicks(inches: Double): Int {
//            return (inches * PARALLEL_TICKS_PER_INCH).toInt()
//        }
//    }
//
//    init {
//        val inverseMatrix = Array2DRowRealMatrix(3, 3)
//        val WHEELS = arrayOf(
//            EncoderWheel(PARALLEL_X, PARALLEL_Y, 0, 0),  // parallel
//            EncoderWheel(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90.0), 1)
//        )
//        for ((x1, y1, heading, row) in WHEELS) {
//            val x = Math.cos(heading)
//            val y = Math.sin(heading)
//            inverseMatrix.setEntry(row, 0, x)
//            inverseMatrix.setEntry(row, 1, y)
//            inverseMatrix.setEntry(
//                row, 2,
//                x1 * y - y1 * x
//            )
//        }
//        inverseMatrix.setEntry(2, 2, 1.0)
//        forwardSolver = LUDecomposition(inverseMatrix).solver
//        require(forwardSolver.isNonSingular) { "The specified configuration cannot support full localization" }
//        prevWheelPositions = IntArray(2) // Initializes with zeros
//        poseUpdate = Pose(start.x, start.y, start.h)
//        relativeRobotMovement = Pose(0, 0, 0)
//    }
}