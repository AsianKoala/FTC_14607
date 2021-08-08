package org.firstinspires.ftc.teamcode.control.system

import android.annotation.SuppressLint
import android.os.Build
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.BNO055IMUImpl
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.hardware.* // ktlint-disable no-wildcard-imports
import com.qualcomm.robotcore.util.Range
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.teamcode.BuildConfig
import org.firstinspires.ftc.teamcode.control.localization.DriftOdo
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.PathPoint
import org.firstinspires.ftc.teamcode.hardware.DriveTrain
import org.firstinspires.ftc.teamcode.hardware.Hardware
import org.firstinspires.ftc.teamcode.util.* // ktlint-disable no-wildcard-imports
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor
import org.openftc.revextensions2.RevBulkData
import java.text.SimpleDateFormat
import java.util.Date
import java.util.LinkedList
import java.util.function.Consumer
import kotlin.collections.ArrayList
import kotlin.math.PI
import kotlin.math.sign

@Config
@Disabled
abstract class Azusa : TunableOpMode() {
    abstract fun startPose(): Pose
    abstract fun path(): Path?

    lateinit var currPose: Pose
    private lateinit var currVel: Pose

    lateinit var pathCache: Path

    private lateinit var masterHub: ExpansionHubEx
    private lateinit var slaveHub: ExpansionHubEx
    private lateinit var masterBulkData: RevBulkData
    private lateinit var slaveBulkData: RevBulkData

    private lateinit var imu: BNO055IMU
    private lateinit var headingOffset: Angle
    private lateinit var odometry: DriftOdo

    private lateinit var frontLeft: ExpansionHubMotor
    private lateinit var frontRight: ExpansionHubMotor
    private lateinit var backLeft: ExpansionHubMotor
    private lateinit var backRight: ExpansionHubMotor

    lateinit var driveTrain: DriveTrain
    private lateinit var allHardware: ArrayList<Hardware>

    private lateinit var dashboard: FtcDashboard
    lateinit var packet: DataPacket

    private lateinit var initTime: Mar
    private lateinit var initLoopTime: Mar
    private lateinit var loopTime: Mar
    private lateinit var odoTime: Mar
    private lateinit var telemTime: Mar
    private lateinit var pathTime: Mar
    private lateinit var dashTime: Mar
    private var maxLT = 0.0

    private var lastManualUpdate: Long = 0
    private var lastAutoUpdate: Long = 0
    private var debugging = false
    private var pathStopped = false
    private lateinit var type: OpModeType
    private lateinit var debugSpeeds: Pose

    override fun init() {
        initTime = Mar()
        initTime.start()
        initLoopTime = Mar()
        loopTime = Mar()
        odoTime = Mar()
        telemTime = Mar()
        pathTime = Mar()
        dashTime = Mar()
        maxLT = -1.0

        masterHub = hardwareMap.get(ExpansionHubEx::class.java, "masterHub")
        slaveHub = hardwareMap.get(ExpansionHubEx::class.java, "slaveHub")
        masterBulkData = masterHub.bulkInputData
        slaveBulkData = slaveHub.bulkInputData

        imu = hardwareMap.get(BNO055IMUImpl::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        parameters.loggingEnabled = false
        imu.initialize(parameters)
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN)
        headingOffset = Angle(imu.angularOrientation.firstAngle.toDouble())

        odometry = DriftOdo(startPose())
        currPose = startPose()
        currVel = Pose(Point(), Angle(Angle.Unit.RAW))

        frontLeft = hardwareMap.get(ExpansionHubMotor::class.java, "FL")
        frontRight = hardwareMap.get(ExpansionHubMotor::class.java, "FR")
        backLeft = hardwareMap.get(ExpansionHubMotor::class.java, "BL")
        backRight = hardwareMap.get(ExpansionHubMotor::class.java, "BR")

        driveTrain = DriveTrain(frontLeft, frontRight, backLeft, backRight)

        allHardware = ArrayList()
        allHardware.add(driveTrain)

        setInternalPath(path())

        lastManualUpdate = System.currentTimeMillis()
        lastAutoUpdate = System.currentTimeMillis()
        debugging = javaClass.isAnnotationPresent(Debuggable::class.java)
        pathStopped = true
        type =
            if (javaClass.isAnnotationPresent(Autonomous::class.java)) OpModeType.AUTO else OpModeType.TELEOP
        debugSpeeds = Pose(Point(), Angle(Angle.Unit.RAW))

        dashboard = FtcDashboard.getInstance()
        initTime.stop()
        updateTelemetry(true)
    }

    override fun init_loop() {
        initLoopTime.start()
        updateTelemetry(true)
        initLoopTime.stop()
    }

    override fun start() {
        telemetry.clear()
    }

    override fun loop() {
        telemetry.clear()
        loopTime.start()
        odoTime.start()
        if (debugging) debugControl() else updateOdo()
        odoTime.stop()
        telemTime.start()
        updateTelemetry(false)
        telemTime.stop()
        pathTime.start()
        updatePath()
        pathTime.stop()
        dashTime.start()
        //        updateDashboard();
        stupidTelemetry()
        dashTime.stop()
        loopTime.stop()
    }

    private fun stupidTelemetry() {
        telemetry.addData("x", currPose.x)
        telemetry.addData("y", currPose.y)
        telemetry.addData("h", currPose.h.deg)
    }

    private fun setInternalPath(path: Path?) {
        pathCache = path ?: LinkedList<PathPoint>() as Path
    }

    private fun updatePath() {
        if (!pathCache.isEmpty()) {
            pathCache.follow(this)
        }
        if (pathCache.finished() && type === OpModeType.AUTO) {
            driveTrain.powers = Pose(Point(), Angle(Angle.Unit.RAW))
        }
    }

    private fun updateOdo() {
        masterBulkData = masterHub.bulkInputData
        slaveBulkData = slaveHub.bulkInputData
        val lastHeading = Angle(imu.angularOrientation.firstAngle.toDouble()) - headingOffset
        currPose =
            odometry.update(this, lastHeading + startPose().h, masterBulkData)
    }

    private fun updateTelemetry(isInit: Boolean) {
        packet = DataPacket()
        packet.addLine(" FTC 14607 by Neil Mehra")
        if (isInit) {
            val buildDate = Date(BuildConfig.TIMESTAMP)
            @SuppressLint("SimpleDateFormat") val dateFormat = SimpleDateFormat("MM/dd HH:mm:ss")
            packet.addData("built at", dateFormat.format(buildDate))
            packet.addData("ftc sdk ver", BuildConfig.VERSION_NAME)
            packet.addLine(Build.MANUFACTURER + " " + Build.MODEL + " running android sdk " + Build.VERSION.SDK_INT)
            packet.addData("hub ver", masterHub.firmwareVersion)
            val revHubs = hardwareMap.getAll(
                LynxModule::class.java
            )
            val motors = hardwareMap.getAll(DcMotor::class.java)
            val servos = hardwareMap.getAll(Servo::class.java)
            val digital = hardwareMap.getAll(
                DigitalChannel::class.java
            )
            val analog = hardwareMap.getAll(
                AnalogInput::class.java
            )
            val i2c = hardwareMap.getAll(I2cDevice::class.java)
            packet.addLine(
                revHubs.size.toString() + " Hubs; " + motors.size + " Motors; " + servos.size +
                    " Servos; " + (digital.size + analog.size + i2c.size) + " Sensors"
            )
            packet.addSpace()
        }
        packet.addData("init time", initTime.time)
        packet.addData("init loop time", initLoopTime.time)
        packet.addData("loop time", loopTime.time)
        if (loopTime.time > maxLT) {
            maxLT = loopTime.time
        }
        packet.addData("max loop time", maxLT)
        packet.addData("hw time", odoTime.time)
        packet.addData("telem time", telemTime.time)
        packet.addData("path time", pathTime.time)
        packet.addData("dash time", dashTime.time)
        packet.addSpace()
        packet.addData("debugging", debugging)
        packet.addData("debug path stopped", pathStopped)
        packet.addSpace()
        packet.addData("x", currPose.x)
        packet.addData("y", currPose.y)
        packet.addData("h", currPose.h.deg)
        packet.addData("odometry", odometry)
        allHardware.forEach(
            Consumer { h: Hardware ->
                h.update(
                    packet
                )
            }
        )
        packet.addSpace()
        if (pathCache.isEmpty()) {
            packet.addData("path empty", "")
        } else {
            packet.addData("path size", pathCache.size)
            packet.addData("current path name", pathCache.name)
            packet.addData("current target", pathCache[0].toString())
            val x = DoubleArray(pathCache.initialPoints.size)
            val y = DoubleArray(pathCache.initialPoints.size)

            for ((index, e) in pathCache.initialPoints.withIndex()) {
                x[index] = e.p.y
                y[index] = -e.p.x
                packet.fieldOverlay().setFill("green").fillCircle(x[index], y[index], 2.0)
            }
            packet.fieldOverlay()
                .setStroke("red")
                .setStrokeWidth(1)
                .strokePolyline(x, y)
        }
//        val (x, y) = currPose.p.dbNormalize
        val (x,y) = Point()
        packet.fieldOverlay()
            .setFill("blue")
            .fillCircle(x, y, 3.0)
            .setStroke("purple")
            .setStrokeWidth(1)
            .strokeLine(
                x, y, x + 10 * currPose.h.sin, y - 10 * currPose.h.cos
            )
        if (isInit) updateDashboard()
    }

    private fun updateDashboard() {
        dashboard.sendTelemetryPacket(packet)
    }

    private fun debugControl() {
        if (gamepad1.left_trigger > 0.5) {
            pathStopped = true
        } else if (gamepad1.right_trigger > 0.5) {
            pathStopped = false
        }
        if (System.currentTimeMillis() - lastManualUpdate > 50) {
            currPose = Pose(
                Point(
                    currPose.x + gamepad1.left_stick_x.sign,
                    currPose.y - gamepad1.left_stick_y.sign
                ),
                (currPose.h + Angle((-gamepad1.right_stick_x).sign.toDouble()).times(Math.PI / 10))
            )
            lastManualUpdate = System.currentTimeMillis()
        }

        if (debugging && !pathStopped && !pathCache.isEmpty()) {
            val elapsed = (System.currentTimeMillis() - lastAutoUpdate) / 1000.0
            lastAutoUpdate = System.currentTimeMillis()
            if (elapsed > 1) return
            val radius: Double = debugSpeeds.hypot
            val theta: Angle = currPose.h + debugSpeeds.p.atan2 - Angle(PI / 2)
            currPose.p += Point(
                radius * theta.cos * elapsed * 500 * 0.2,
                radius * theta.sin * elapsed * 500 * 0.2
            )
            currPose.h.angle += driveTrain.powers.h.angle * elapsed * 10 / (2 * Math.PI)

            debugSpeeds.p.x += Range.clip(
                (driveTrain.powers.x - debugSpeeds.x) / 0.2,
                -1.0,
                1.0
            ) * elapsed
            debugSpeeds.p.y += Range.clip(
                (driveTrain.powers.y - debugSpeeds.y) / 0.2,
                -1.0,
                1.0
            ) * elapsed
            debugSpeeds.h.angle += Range.clip(
                ((driveTrain.powers.h - debugSpeeds.h) / 0.2).raw,
                -1.0,
                1.0
            ) * elapsed
            debugSpeeds.p *= (1.0 - elapsed)
            debugSpeeds.h.angle *= (1.0 - elapsed)
        }
    }
}
