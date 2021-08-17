package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.BNO055IMUImpl
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.control.localization.Speedometer
import org.firstinspires.ftc.teamcode.control.localization.ThreeWheelOdometry
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.MathUtil.toDegrees
import org.firstinspires.ftc.teamcode.util.MathUtil.wrap
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor
import org.openftc.revextensions2.RevBulkData
import kotlin.collections.ArrayList
import kotlin.math.sign

@Config
class Azusa(val startPose: Pose, val debugging: Boolean) {

    lateinit var currPose: Pose
    lateinit var currVel: Pose

    lateinit var masterHub: ExpansionHubEx
    lateinit var masterBulkData: RevBulkData

    lateinit var odometry: ThreeWheelOdometry

    lateinit var driveTrain: DriveTrain
    lateinit var allHardware: ArrayList<Hardware>

    lateinit var dashboard: FtcDashboard

    private var lastManualUpdate: Long = 0
    private var lastAutoUpdate: Long = 0
    private var pathStopped = false
    private lateinit var debugSpeeds: Pose

    lateinit var azuTelemetry: AzusaTelemetry

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

        currPose = Pose(Point.ORIGIN, Angle(0.0, Angle.Unit.RAW))
        currVel = Pose(Point.ORIGIN, Angle(0.0, Angle.Unit.RAW))

        driveTrain = DriveTrain(
            hwMap.get(ExpansionHubMotor::class.java, "FL"),
            hwMap.get(ExpansionHubMotor::class.java, "FR"),
            hwMap.get(ExpansionHubMotor::class.java, "BL"),
            hwMap.get(ExpansionHubMotor::class.java, "BR")
        )
        allHardware.add(driveTrain)

        debugSpeeds = Pose(Point.ORIGIN, Angle(0.0, Angle.Unit.RAW))
        lastManualUpdate = System.currentTimeMillis()
        lastAutoUpdate = System.currentTimeMillis()
        pathStopped = true

        dashboard = FtcDashboard.getInstance()
    }

    fun update() {
        if (!debugging) updateOdo()
        updateHW()
        updateTelemetry()
    }

    private fun updateTelemetry() {
        azuTelemetry.addData("x", currPose.x)
        azuTelemetry.addData("y", currPose.y)
        azuTelemetry.addData("h", currPose.h.deg)
        azuTelemetry.addData("powers", driveTrain.powers.toRawString)

        val (x, y) = currPose.p.dbNormalize
        azuTelemetry.fieldOverlay()
            .setFill("blue")
            .fillCircle(x, y, 3.0)
            .setStroke("purple")
            .setStrokeWidth(1)
            .strokeLine(
                x, y, x + 10 * currPose.h.sin, y - 10 * currPose.h.cos
            )
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

        currVel = Speedometer.update(currPose.h)
    }

    private fun updateHW() {
        allHardware.forEach { it.update(azuTelemetry) }
    }

    fun teleopControl(gamepad: Gamepad, driveScale: Double) {
        azuTelemetry.addData("ls x", gamepad.left_stick_x)
        azuTelemetry.addData("ls y", -gamepad.left_stick_y)
        azuTelemetry.addData("rs x", -gamepad.right_stick_x)

//        val driveScale = 0.45 - if (gamepad.left_bumper) 0.2 else 0.0
        driveTrain.powers = Pose(
            Point(
                gamepad.left_stick_x * driveScale,
                -gamepad.left_stick_y * driveScale
            ),
            Angle(-gamepad.right_stick_x * driveScale, Angle.Unit.RAW)
        )
        driveTrain.update(azuTelemetry)
    }

    fun debugControl(gamepad: Gamepad) {
        if (gamepad.left_trigger > 0.5) {
            pathStopped = true
        } else if (gamepad.right_trigger > 0.5) {
            pathStopped = false
        }
        if (System.currentTimeMillis() - lastManualUpdate > 50) {
            currPose = Pose(
                Point(
                    currPose.x + gamepad.left_stick_x.sign,
                    currPose.y - gamepad.left_stick_y.sign
                ),
                (currPose.h + Angle((-gamepad.right_stick_x).sign.toDouble(), Angle.Unit.RAW).times(Math.PI / 10))
            )
            lastManualUpdate = System.currentTimeMillis()
        }

        if (!pathStopped) {
            val elapsed = (System.currentTimeMillis() - lastAutoUpdate) / 1000.0
            lastAutoUpdate = System.currentTimeMillis()
            if (elapsed > 1) return

            val radius: Double = debugSpeeds.hypot
            val theta = currPose.h + debugSpeeds.p.atan2 - Angle(Math.PI / 2, Angle.Unit.RAW)

            currPose.p.x += radius * theta.cos * elapsed * 100
            currPose.p.y += radius * theta.sin * elapsed * 100
            currPose.h += Angle(driveTrain.powers.h.raw * elapsed * 10.0 / (2 * Math.PI), Angle.Unit.RAD)

            debugSpeeds.p.x += (Range.clip((driveTrain.powers.x - debugSpeeds.x) / 0.2, -1.0, 1.0)) * elapsed * (1.0 - elapsed)
            debugSpeeds.p.y += (Range.clip((driveTrain.powers.y - debugSpeeds.y) / 0.2, -1.0, 1.0)) * elapsed * (1.0 - elapsed)
            debugSpeeds.h += Angle(Range.clip((driveTrain.powers.h.raw - debugSpeeds.h.raw) / 0.2, -1.0, 1.0), Angle.Unit.RAW) * elapsed * (1.0 - elapsed)
        }
    }
}
