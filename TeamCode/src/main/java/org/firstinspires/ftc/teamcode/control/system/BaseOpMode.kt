package org.firstinspires.ftc.teamcode.control.system

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode
import org.firstinspires.ftc.teamcode.hardware.Azusa
import org.firstinspires.ftc.teamcode.util.debug.Debuggable
import org.firstinspires.ftc.teamcode.util.math.Pose
import org.firstinspires.ftc.teamcode.util.opmode.AzusaTelemetry
import org.firstinspires.ftc.teamcode.util.opmode.OpModePacket
import org.firstinspires.ftc.teamcode.util.opmode.OpModeType
import org.firstinspires.ftc.teamcode.util.opmode.Status

abstract class BaseOpMode : TunableLinearOpMode() {
    abstract fun startPose(): Pose

    lateinit var azusa: Azusa

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
        azusa = Azusa(OpModePacket(
                startPose(), debugging, hardwareMap, azusaTelemetry, gamepad1, gamepad2
        ))

        opModeType = if (javaClass.isAnnotationPresent(Autonomous::class.java)) {
            OpModeType.AUTO
        } else OpModeType.TELEOP

        azusa.init()
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
            azusa.update()
            azusaTelemetry.update()
        }
        onStop()
//        if (opModeType == OpModeType.AUTO)
//            (internalOpModeServices as OpModeManagerImpl).initActiveOpMode(Globals.TELEOP_NAME)
    }

    open fun onInit() {}
    open fun onInitLoop() {}
    open fun onStart() {}
    abstract fun onLoop()
    open fun onStop() {}
}
