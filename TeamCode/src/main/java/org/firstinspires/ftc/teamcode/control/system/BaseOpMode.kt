package org.firstinspires.ftc.teamcode.control.system

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl
import org.firstinspires.ftc.teamcode.hardware.Azusa
import org.firstinspires.ftc.teamcode.util.AzusaTelemetry
import org.firstinspires.ftc.teamcode.util.debug.Debuggable
import org.firstinspires.ftc.teamcode.util.opmode.OpModeType
import org.firstinspires.ftc.teamcode.util.math.Pose

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
        azusa = Azusa(startPose(), debugging)

        opModeType = when (javaClass.isAnnotationPresent(Autonomous::class.java)) {
            true -> OpModeType.AUTO
            false -> OpModeType.TELEOP
        }

        azusa.init(hardwareMap, azusaTelemetry)
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
        if (opModeType == OpModeType.AUTO)
            (internalOpModeServices as OpModeManagerImpl).initActiveOpMode("AzusaNewTeleOp")
    }

    open fun onInit() {}
    open fun onInitLoop() {}
    open fun onStart() {}
    abstract fun onLoop()
    open fun onStop() {}
}
