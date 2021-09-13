package org.firstinspires.ftc.teamcode.control.system

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl
import org.firstinspires.ftc.teamcode.hardware.Azusa
import org.firstinspires.ftc.teamcode.hardware.BaseRobot
import org.firstinspires.ftc.teamcode.hardware.Firefly
import org.firstinspires.ftc.teamcode.util.debug.Debuggable
import org.firstinspires.ftc.teamcode.util.math.Pose
import org.firstinspires.ftc.teamcode.util.opmode.AzusaTelemetry
import org.firstinspires.ftc.teamcode.util.opmode.Globals
import org.firstinspires.ftc.teamcode.util.opmode.OpModePacket
import org.firstinspires.ftc.teamcode.util.opmode.OpModeType
import org.firstinspires.ftc.teamcode.util.opmode.Status

abstract class BaseOpMode : TunableLinearOpMode() {
    abstract val startPose: Pose

    private lateinit var robot: BaseRobot
    lateinit var azusa: Azusa
    lateinit var firefly: Firefly

    lateinit var azusaTelemetry: AzusaTelemetry

    private val status: Status
        get() = when {
            isStopRequested -> Status.STOP
            isStarted -> Status.LOOP
            else -> Status.INIT_LOOP
        }

    private var hasStarted = false

    lateinit var opModeType: OpModeType
    var debugging = false

    override fun runOpMode() {
        debugging = javaClass.isAnnotationPresent(Debuggable::class.java)

        azusaTelemetry = AzusaTelemetry(this)


        val packet = OpModePacket(startPose, debugging, hardwareMap, azusaTelemetry, gamepad1, gamepad2)
        if ((internalOpModeServices as OpModeManagerImpl).activeOpModeName.startsWith("Azusa", true)) {
            azusa = Azusa(packet)
            robot = azusa

        } else {
            firefly = Firefly(packet)
            robot = firefly
        }

        opModeType = if (javaClass.isAnnotationPresent(Autonomous::class.java)) {
            OpModeType.AUTO
        } else OpModeType.TELEOP

        robot.init()
        onInit()

        mainLoop@ while (true) {
            when (status) {
                Status.INIT_LOOP -> {
                    onInitLoop()
                }

                Status.LOOP -> {
                    if (hasStarted) {
                        onLoop()
                    } else {
                        onStart()
                        hasStarted = true
                    }
                }

                Status.STOP -> {
                    break@mainLoop
                }
            }
            robot.update()
            azusaTelemetry.update()
        }

        onStop()
        if (opModeType == OpModeType.AUTO && Globals.IS_COMP)
            (internalOpModeServices as OpModeManagerImpl).initActiveOpMode(Globals.TELEOP_NAME)
    }

    open fun onInit() {}
    open fun onInitLoop() {}
    open fun onStart() {}
    abstract fun onLoop()
    open fun onStop() {}
}
