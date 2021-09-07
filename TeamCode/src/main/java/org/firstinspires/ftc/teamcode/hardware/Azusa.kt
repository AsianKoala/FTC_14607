package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.control.localization.ThreeWheelOdometry
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.MathUtil.toRadians
import org.firstinspires.ftc.teamcode.util.math.Point
import org.firstinspires.ftc.teamcode.util.math.Pose
import org.firstinspires.ftc.teamcode.util.opmode.AzusaTelemetry
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor
import org.openftc.revextensions2.RevBulkData
import java.util.*
import kotlin.collections.ArrayList
import kotlin.math.*

@Config
class Azusa(val startPose: Pose, private val debugging: Boolean) {

    lateinit var currPose: Pose
//    lateinit var currVel: Pose

    lateinit var masterHub: ExpansionHubEx
    lateinit var masterBulkData: RevBulkData

    lateinit var odometry: ThreeWheelOdometry

    lateinit var driveTrain: DriveTrain
    lateinit var allHardware: ArrayList<Hardware>

    lateinit var dashboard: FtcDashboard
    lateinit var azuTelemetry: AzusaTelemetry

    private val lastPositions = LinkedList<Point>()
    var lastllupdate: Long = 0

    var xSpeed = 0.0
    var ySpeed = 0.0
    var turnSpeed = 0.0

    var lastUpdateTime: Long = 0

    fun init(hwMap: HardwareMap, telemetry: AzusaTelemetry) {
        azuTelemetry = telemetry

        allHardware = ArrayList()

        masterHub = hwMap.get(ExpansionHubEx::class.java, "masterHub")
        masterBulkData = masterHub.bulkInputData

        odometry = ThreeWheelOdometry(
            startPose,
            masterBulkData.getMotorCurrentPosition(1),
            masterBulkData.getMotorCurrentPosition(3),
            masterBulkData.getMotorCurrentPosition(2)
        )

        currPose = startPose

//        currVel = Pose(Point.ORIGIN, Angle(0.0, AngleUnit.RAW))

        driveTrain = DriveTrain(
            hwMap.get(ExpansionHubMotor::class.java, "FL"),
            hwMap.get(ExpansionHubMotor::class.java, "FR"),
            hwMap.get(ExpansionHubMotor::class.java, "BL"),
            hwMap.get(ExpansionHubMotor::class.java, "BR")
        )

        allHardware.add(driveTrain)

        dashboard = FtcDashboard.getInstance()
    }

    fun update() {
        if (!debugging) updateOdo() else debugPhysics()
        updateHW()
        updateTelemetry()
    }

    private fun updateTelemetry() {
        azuTelemetry.addData("x", currPose.x)
        azuTelemetry.addData("y", currPose.y)
        azuTelemetry.addData("h", currPose.h.deg)
        azuTelemetry.addData("powers", driveTrain.powers.toRawString)

        val (x, y) = currPose.p.dbNormalize
        val radius = 5
        azuTelemetry.fieldOverlay()
            .setFill("blue")
            .fillCircle(x, y, 3.0)
            .setStroke("purple")
            .setStrokeWidth(1)
            .strokeLine(
                x, y, x + radius * currPose.h.sin, y - radius * currPose.h.cos
            )

        if (System.currentTimeMillis() - lastllupdate > 250) {
            lastllupdate = System.currentTimeMillis()
            lastPositions.add(currPose.p.copy)
        }

        val iterator = lastPositions.iterator()
        while (iterator.hasNext()) {
            val curr = iterator.next()
            azuTelemetry.fieldOverlay()
                .setStroke("red")
                .strokeCircle(curr.dbNormalize.x, curr.dbNormalize.y, 1.0)
        }
    }

    private fun updateOdo() {
        if (masterHub.bulkInputData != null) {
            masterBulkData = masterHub.bulkInputData
        }

        currPose = odometry.update(
            azuTelemetry,
            masterBulkData.getMotorCurrentPosition(1),
            masterBulkData.getMotorCurrentPosition(3),
            masterBulkData.getMotorCurrentPosition(2)
        )
    }

    private fun updateHW() {
        allHardware.forEach { it.update(azuTelemetry) }
    }

    fun teleopControl(gamepad: Gamepad, driveScale: Double, fieldOriented: Boolean) {
        if (fieldOriented) {
            val p = Point(gamepad.left_stick_x.toDouble(), -gamepad.left_stick_y.toDouble())
            val h = p.hypot
            val angle = p.atan2

            driveTrain.powers = Pose(
                Point(
                    h * angle.cos,
                    h * angle.sin
                ),
                Angle(-gamepad.right_stick_x.toDouble(), AngleUnit.RAW)
            )
        } else {
            driveTrain.powers = Pose(
                Point(
                    gamepad.left_stick_x * driveScale,
                    -gamepad.left_stick_y * driveScale
                ),
                Angle(-gamepad.right_stick_x * driveScale, AngleUnit.RAW)
            )
            driveTrain.update(azuTelemetry)
        }
    }

    private fun debugPhysics() {
        currPose.h = currPose.h.wrap()

        // current time
        val currentTimeMillis = System.currentTimeMillis()
        // elapsed time
        val elapsedTime = (currentTimeMillis - lastUpdateTime) / 1000.0

        lastUpdateTime = currentTimeMillis
        if (elapsedTime > 1) {
            println("you fucked up")
            return
        }

        val totalSpeed = hypot(xSpeed, ySpeed)
        val angle = atan2(ySpeed, xSpeed) - 90.0.toRadians
        val outputAngle = currPose.h.angle + angle
        currPose.p.x += totalSpeed * cos(outputAngle) * elapsedTime * 40
        currPose.p.y += totalSpeed * sin(outputAngle) * elapsedTime * 40

        currPose.h.angle += turnSpeed * elapsedTime * 40 / (2 * PI)

        xSpeed += Range.clip((driveTrain.powers.x - xSpeed) / 0.2, -1.0, 1.0) * elapsedTime
        ySpeed += Range.clip((driveTrain.powers.y - ySpeed) / 0.2, -1.0, 1.0) * elapsedTime
        turnSpeed += Range.clip((driveTrain.powers.h.angle - turnSpeed) / 0.2, -1.0, 1.0) * elapsedTime

        xSpeed *= 1.0 - (elapsedTime)
        ySpeed *= 1.0 - (elapsedTime)
        turnSpeed *= 1.0 - (elapsedTime)
    }
}
