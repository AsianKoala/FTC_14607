package org.firstinspires.ftc.teamcode.control.localization

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.Pose

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
/*
center of rotation is (8.75, 6.25)
perp wheel is (4.5, 13)
parallel wheel is (13, 13)
deltas are as follows:
perp wheel: (-4.25, 6.75)
parallel wheel: (4.25, 6.75)
reverse the coords and we get this
perp: (6.75, -4.25)
parallel (6.75, 4.25)

 */
@Deprecated("never coming back to this piece of shit")
class WorkingOdometry(hardwareMap: HardwareMap, startPose: Pose) {
//    Arrays.asList(
//        Pose2d(PARALLEL_X, PARALLEL_Y, 0),
//        Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90.0))
//    )
//) {
//    private var currPose: Pose
//    private var currVel: Pose
//    private val prevPoses: ArrayList<TimePose>
//
//    // Parallel wheel is parallel to the forward axis
//    // Perpendicular is perpendicular to the forward axis
//    private val parallelEncoder: Encoder
//    private val perpendicularEncoder: Encoder
//    override fun getHeading(): Double {
//        return currPose.h
//    }
//
//    override fun getWheelPositions(): List<Double> {
//        return Arrays.asList(
//            encoderTicksToInches(parallelEncoder.currentPosition.toDouble()),
//            encoderTicksToInches(perpendicularEncoder.currentPosition.toDouble())
//        )
//    }
//
//    override fun getWheelVelocities(): List<Double> {
//        return Arrays.asList(
//            encoderTicksToInches(parallelEncoder.correctedVelocity),
//            encoderTicksToInches(perpendicularEncoder.correctedVelocity)
//        )
//    }
//
//    override fun update() {}
//    fun realUpdate(angle: Double): Array<Pose> {
//        super.update()
//        currPose.h = angle
//        currPose = Pose(poseEstimate)
//        prevPoses.add(TimePose(currPose, System.currentTimeMillis()))
//        if (prevPoses.size > 1) {
//            val oldIndex = Math.max(0, prevPoses.size - 6)
//            val old = prevPoses[oldIndex]
//            val cur = prevPoses[prevPoses.size - 1]
//            val scale = (cur.sign - old.sign) as Double / 1000
//            currVel = cur.minus(old).scale(1 / scale)
//        }
//        return arrayOf(currPose, currVel)
//    }
//
//    @SuppressLint("DefaultLocale")
//    override fun toString(): String {
//        return String.format(
//            "(%.1f, %.1f, %.1f)",
//            currPose.x,
//            currPose.y,
//            Math.toDegrees(currPose.h)
//        )
//    }
//
//    companion object {
//        const val TICKS_PER_INCH = 1103.8839
//
//        // 6.75, 4.25, 6.75, -4.25
//        //    public static double PARALLEL_X = -6.75; // X is the up and down direction
//        //    public static double PARALLEL_Y = -4.25; // Y is the strafe direction
//        //
//        //    public static double PERPENDICULAR_X = -6.75;
//        //    public static double PERPENDICULAR_Y = 4.25;
//        private const val PARALLEL_X = 6.75
//        private const val PARALLEL_Y = 4.25
//        private const val PERPENDICULAR_X = 6.75
//        private const val PERPENDICULAR_Y = -4.25
//        fun encoderTicksToInches(ticks: Double): Double {
//            return ticks / TICKS_PER_INCH
//        }
//    }
//
//    init {
//        parallelEncoder = Encoder(
//            hardwareMap.get(
//                DcMotorEx::class.java, "leftIntake"
//            )
//        )
//        perpendicularEncoder = Encoder(
//            hardwareMap.get(
//                DcMotorEx::class.java, "rightIntake"
//            )
//        )
//        parallelEncoder.direction = Encoder.Direction.REVERSE
//        perpendicularEncoder.direction = Encoder.Direction.REVERSE
//        currPose = Pose(startPose)
//        poseEstimate = Pose2d(startPose.x, startPose.y, startPose.h)
//        currVel = Pose()
//        prevPoses = ArrayList()
//        prevPoses.add(TimePose(startPose, System.currentTimeMillis()))
//    }
}