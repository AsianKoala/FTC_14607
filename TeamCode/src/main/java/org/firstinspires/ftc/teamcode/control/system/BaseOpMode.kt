package org.firstinspires.ftc.teamcode.control.system

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode
import org.firstinspires.ftc.teamcode.hardware.Azusa
import org.firstinspires.ftc.teamcode.util.MarkerNew
import org.firstinspires.ftc.teamcode.util.OpModeType

abstract class BaseOpMode : TunableLinearOpMode() {
    abstract val robot: Azusa

    lateinit var opModeType: OpModeType
    private var status: Status = Status.INIT_LOOP

    override fun runOpMode() {
        val mar = MarkerNew("init")
        opModeType = when (javaClass.isAnnotationPresent(Autonomous::class.java)) {
            true -> OpModeType.AUTO
            false -> OpModeType.TELEOP
        }

        robot.init()
        onInit()
        mar.stop()
        telemetry.addData("init time", mar.time)
        telemetry.update()

        mainLoop@ while (true) {
            mar.name = status.name
            mar.start()
            when (status) {
                Status.INIT_LOOP -> {
                    onInitLoop()
                    if (isStarted)
                        status = status.next
                }

                Status.START -> {
                    onStart()
                    status = status.next
                }

                Status.LOOP -> {
                    onLoop()
                    if (isStopRequested)
                        status = status.next
                }

                Status.STOP -> {
                    onStop()
                    break@mainLoop
                }
            }
            robot.update()
            mar.stop()
            telemetry.addData(status.name + " time", mar.time)
            telemetry.update()
        }
    }

    open fun onInit() {}
    open fun onInitLoop() {}
    open fun onStart() {}
    abstract fun onLoop()
    open fun onStop() {}
}
