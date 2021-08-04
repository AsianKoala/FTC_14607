package org.firstinspires.ftc.teamcode.control.localization

// used to get accurate wheel deltas for odom implementations
@Deprecated("never planning to go back to this piece of shit")
abstract class BaseOdometry() {
//    protected var wheelDeltaScaled: Pose
//    protected var currPoseDelta: Pose
//    protected var currPose: Pose
//    private var currVelocity: Pose
//    private var prevWheelPositions: Pose
//    private val prevPoses: ArrayList<TimePose>
//    private var robotPoseDelta: Pose
//    private val forwardSolver: DecompositionSolver
//
//    protected fun calcPoseDeltas(deltas: Pose?): Pose {
//        val m = MatrixUtils.createRealMatrix(
//            arrayOf(
//                doubleArrayOf(
//                    deltas!!.x,
//                    deltas.y,
//                    deltas.h
//                )
//            )
//        )
//        val rawPoseDelta = forwardSolver.solve(m.transpose())
//        return Pose(
//            rawPoseDelta.getEntry(0, 0),
//            rawPoseDelta.getEntry(1, 0),
//            rawPoseDelta.getEntry(2, 0)
//        )
//    }
//
//    private fun calcRobotVel() {
//        if (prevPoses.size > 1) {
//            val oldIndex = Math.max(0, prevPoses.size - 2 - 1)
//            val old = prevPoses[oldIndex]
//            val cur = prevPoses[prevPoses.size - 1]
//            val scale = (cur.sign - old.sign) as Double / 1000
//            currVelocity = Pose(cur.minus(old).scale(1 / scale))
//        } else {
//            currVelocity = Pose(0, 0, 0)
//        }
//    }
//
//    // should only be used for start of auto
//    fun setStartingPose(startPose: Pose?) {
//        currPose = startPose
//        prevWheelPositions = Pose()
//    }
//
//    protected abstract fun robotPoseUpdate()
//    fun update(wheelPositions: Pose): Array<Pose?> { // todo fix vel
//        wheelPositions.minus(prevWheelPositions)
//        wheelDeltaScaled = wheelPositions.scale(TICKS_PER_INCH)
//        currPoseDelta = calcPoseDeltas(wheelDeltaScaled)
//        val oldPose = currPose
//        robotPoseUpdate()
//        currPose = currPose.wrap()
//        val deltaVals: Pose = currPose.minus(oldPose)
//        robotPoseDelta = Pose(robotPoseDelta.add(deltaVals))
//        prevPoses.add(TimePose(currPose!!, System.currentTimeMillis()))
//        calcRobotVel()
//        prevWheelPositions = Pose(currPose)
//        return arrayOf(currPose, currVelocity, currPoseDelta)
//    }
//
//    companion object {
//        private const val TICKS_PER_INCH = 1103.8839
//        private const val LATERAL_DISTANCE = 10.0
//        private const val HORIZONTAL_WHEEL_OFFSET = 10.0
//    }
//
//    init {
//        setStartingPose(startPose)
//        val inverseMatrix = Array2DRowRealMatrix(3, 3)
//        val wheelPoses = arrayOf(
//            Pose(0, LATERAL_DISTANCE / 2, 0),
//            Pose(0, -LATERAL_DISTANCE / 2, 0),
//            Pose(HORIZONTAL_WHEEL_OFFSET, 0, Math.toRadians(90.0))
//        )
//        for (i in 0..2) {
//            val currentWheelPose = wheelPoses[i]
//            val x = currentWheelPose.cos
//            val y = currentWheelPose.sin
//            inverseMatrix.setEntry(i, 0, x)
//            inverseMatrix.setEntry(i, 1, y)
//            inverseMatrix.setEntry(
//                i, 2,
//                currentWheelPose.x * y - currentWheelPose.y * x
//            )
//        }
//        forwardSolver = LUDecomposition(inverseMatrix).solver
//        robotPoseDelta = Pose()
//        prevPoses = ArrayList()
//        prevPoses.add(TimePose(robotPoseDelta, System.currentTimeMillis()))
//    }
}