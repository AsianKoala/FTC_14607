package org.firstinspires.ftc.teamcode.control.system

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode
import org.firstinspires.ftc.teamcode.hardware.Azusa
import org.firstinspires.ftc.teamcode.util.Debuggable
import org.firstinspires.ftc.teamcode.util.MarkerNew
import org.firstinspires.ftc.teamcode.util.OpModeType
import org.firstinspires.ftc.teamcode.util.Pose

abstract class BaseOpMode : TunableLinearOpMode() {
    abstract fun startPose(): Pose

    lateinit var azusa: Azusa

    private val status: Status
        get() = when {
            isStarted -> Status.LOOP
            isStopRequested -> Status.STOP
            else -> Status.INIT_LOOP
        }

    private var hasStarted = false

    lateinit var opModeType: OpModeType
    var debugging = false

    override fun runOpMode() {
        val mar = MarkerNew("init")

        debugging = javaClass.isAnnotationPresent(Debuggable::class.java)

        azusa = Azusa(startPose(), debugging)

        opModeType = when (javaClass.isAnnotationPresent(Autonomous::class.java)) {
            true -> OpModeType.AUTO
            false -> OpModeType.TELEOP
        }

        azusa.init(hardwareMap, telemetry)
        onInit()

        telemetry.addData("init time", mar.time)
        telemetry.update()

        mainLoop@ while (true) {
            mar.name = status.name
            mar.start()

            when (status) {
                Status.INIT_LOOP -> {
                    onInitLoop()
                }

                Status.LOOP -> {
                    if (!hasStarted) {
                        onStart()
                        hasStarted = true
                    } else
                        onLoop()
                    azusa.update()
                }

                Status.STOP -> {
                    break@mainLoop
                }
            }
            telemetry.addData(status.name + " time", mar.time)
            telemetry.update()
        }
        onStop()
    }

    open fun onInit() {}
    open fun onInitLoop() {}
    open fun onStart() {}
    abstract fun onLoop()
    open fun onStop() {}
}
