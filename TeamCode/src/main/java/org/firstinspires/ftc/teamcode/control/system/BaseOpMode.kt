package org.firstinspires.ftc.teamcode.control.system

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.BaseAzusa
import org.firstinspires.ftc.teamcode.util.AllianceSide
import org.firstinspires.ftc.teamcode.util.OpModeType

class BaseOpMode(val azusa: BaseAzusa, val allianceSide: AllianceSide) : LinearOpMode() {

    lateinit var opModeType: OpModeType
    private var status: Status = Status.INIT_LOOP

    override fun runOpMode() {
        onInit()

        mainLoop@ while(true) {
            when(status) {
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
        }
    }

    open fun onInit() {}
    open fun onInitLoop() {}
    open fun onStart() {}
    open fun onLoop() {}
    open fun onStop() {}
}
