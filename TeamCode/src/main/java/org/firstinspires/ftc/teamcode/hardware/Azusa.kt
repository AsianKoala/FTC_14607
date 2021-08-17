package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.BNO055IMUImpl
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
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

    lateinit var imu: BNO055IMU
    var headingOffset: Double = 0.0

    lateinit var driveTrain: DriveTrain
    lateinit var allHardware: ArrayList<Hardware>

    lateinit var dashboard: FtcDashboard
    lateinit var packet: DataPacket

    private var lastManualUpdate: Long = 0
    private var lastAutoUpdate: Long = 0
    private var pathStopped = false
    private lateinit var debugSpeeds: Pose

    lateinit var telemetry: Telemetry

    fun init(hwMap: HardwareMap, telemetry: Telemetry) {
        this.telemetry = telemetry

        allHardware = ArrayList()

        masterHub = hwMap.get(ExpansionHubEx::class.java, "masterHub")
        masterBulkData = masterHub.bulkInputData

        odometry = ThreeWheelOdometry(
            startPose,
            masterBulkData.getMotorCurrentPosition(1),
            masterBulkData.getMotorCurrentPosition(3),
            masterBulkData.getMotorCurrentPosition(2)
        )

        imu = hwMap.get(BNO055IMUImpl::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        parameters.loggingEnabled = false
        imu.initialize(parameters)
        headingOffset = imu.angularOrientation.firstAngle.toDouble()

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
        packet = DataPacket()
    }

    fun update() {
        if (!debugging) updateOdo()
        updateHW()
        updateTelemetry()
    }

    private fun updateTelemetry() {
        telemetry.addData("x", currPose.x)
        telemetry.addData("y", currPose.y)
        telemetry.addData("h", currPose.h.deg)
        telemetry.addData("powers", driveTrain.powers.toRawString)
        telemetry.addData("path stopped", pathStopped)
        telemetry.addData("debugging", debugging)
        telemetry.update()
    }

    private fun updateOdo() {
        if(masterHub.bulkInputData != null) {
            masterBulkData = masterHub.bulkInputData
        }

        val lastHeading = (imu.angularOrientation.firstAngle - headingOffset).wrap.toDegrees
        telemetry.addData("imu heading", lastHeading)

        currPose = odometry.update(
            telemetry,
            masterBulkData.getMotorCurrentPosition(1),
            masterBulkData.getMotorCurrentPosition(3),
            masterBulkData.getMotorCurrentPosition(2),
                lastHeading
        )

        currVel = Speedometer.update(currPose.h)
    }

    private fun updateHW() {
        allHardware.forEach { it.update(telemetry) }
    }

    fun teleopControl(gamepad: Gamepad, driveScale: Double) {
        telemetry.addData("ls x", gamepad.left_stick_x)
        telemetry.addData("ls y", -gamepad.left_stick_y)
        telemetry.addData("rs x", -gamepad.right_stick_x)

//        val driveScale = 0.45 - if (gamepad.left_bumper) 0.2 else 0.0
        driveTrain.powers = Pose(Point(
            gamepad.left_stick_x * driveScale,
            -gamepad.left_stick_y * driveScale),
            Angle(-gamepad.right_stick_x * driveScale, Angle.Unit.RAW)
        )
        driveTrain.update(telemetry)
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

            packet.addLine(debugSpeeds.toRawString)
        }
    }
}
