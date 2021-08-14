package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.localization.Speedometer
import org.firstinspires.ftc.teamcode.control.localization.ThreeWheelOdometry
import org.firstinspires.ftc.teamcode.util.*
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor
import org.openftc.revextensions2.RevBulkData
import kotlin.collections.ArrayList
import kotlin.math.sign

@Config
class Azusa(private val startPose: Pose, private val hwMap: HardwareMap, val telemetry: Telemetry) {

    lateinit var currPose: Pose
    lateinit var currVel: Pose

    lateinit var masterHub: ExpansionHubEx
    lateinit var masterBulkData: RevBulkData

    lateinit var odometry: ThreeWheelOdometry
    lateinit var speedometer: Speedometer

    lateinit var driveTrain: DriveTrain
    lateinit var allHardware: ArrayList<Hardware>

    lateinit var dashboard: FtcDashboard
    lateinit var packet: DataPacket

    private var lastManualUpdate: Long = 0
    private var lastAutoUpdate: Long = 0
    private var debugging = false
    private var pathStopped = false
    private lateinit var debugSpeeds: Pose

    fun init() {
        allHardware = ArrayList()

        masterHub = hwMap.get(ExpansionHubEx::class.java, "masterHub")
        masterBulkData = masterHub.bulkInputData

        odometry = ThreeWheelOdometry(startPose)
        speedometer = Speedometer()

        driveTrain = DriveTrain(
            hwMap.get(ExpansionHubMotor::class.java, "FL"),
            hwMap.get(ExpansionHubMotor::class.java, "FR"),
            hwMap.get(ExpansionHubMotor::class.java, "BL"),
            hwMap.get(ExpansionHubMotor::class.java, "BR")
        )
        allHardware.add(driveTrain)

        lastManualUpdate = System.currentTimeMillis()
        lastAutoUpdate = System.currentTimeMillis()
        debugging = javaClass.isAnnotationPresent(Debuggable::class.java)
        pathStopped = true

        dashboard = FtcDashboard.getInstance()
        packet = DataPacket()
    }

    fun update() {
        masterBulkData = masterHub.bulkInputData

        currPose = odometry.update(
            masterBulkData.getMotorCurrentPosition(0),
            masterBulkData.getMotorCurrentPosition(1),
            masterBulkData.getMotorCurrentPosition(2)
        )

        currVel = speedometer.update(currPose.h)

        allHardware.forEach { it.update(telemetry) }
    }

    fun teleopControl(gamepad: Gamepad, fieldCentric: Boolean) {
        val scale: Double = if (gamepad.left_bumper) 1.0 else 0.5
        val move = Point(-gamepad.left_stick_x.toDouble(), gamepad.left_stick_y.toDouble())
        val turn = -gamepad.right_stick_x.toDouble()
        var powers = Pose()
        if (fieldCentric) {
            val vel = move.hypot
            val angle = move.atan2 - currPose.h

            powers.p.x = angle.sin * vel
            powers.p.y = angle.cos * vel
            powers.h = Angle(turn, Angle.Unit.RAW)
        } else {
            powers = Pose(move, Angle(-gamepad.right_stick_x * scale, Angle.Unit.RAW))
        }
        driveTrain.powers = powers
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
                (currPose.h + Angle((-gamepad.right_stick_x).sign.toDouble()).times(Math.PI / 10))
            )
            lastManualUpdate = System.currentTimeMillis()
        }

        if (debugging && !pathStopped) {
            val elapsed = (System.currentTimeMillis() - lastAutoUpdate) / 1000.0
            lastAutoUpdate = System.currentTimeMillis()
            if (elapsed > 1) return

            val radius: Double = debugSpeeds.hypot
            val theta = currPose.h + debugSpeeds.p.atan2 - Angle(Math.PI / 2)

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
