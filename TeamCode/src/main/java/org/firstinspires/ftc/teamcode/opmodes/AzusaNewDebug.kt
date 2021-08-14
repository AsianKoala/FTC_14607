package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.system.BaseOpMode
import org.firstinspires.ftc.teamcode.hardware.Azusa
import org.firstinspires.ftc.teamcode.util.Debuggable
import org.firstinspires.ftc.teamcode.util.Pose

@Debuggable
@TeleOp
class AzusaNewDebug : BaseOpMode() {
    val azusa = Azusa(Pose(), hardwareMap, telemetry)

    override val robot: Azusa
        get() = azusa

    override fun onLoop() {
        azusa.debugControl(gamepad1)
    }
}