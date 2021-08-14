package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.system.BaseOpMode
import org.firstinspires.ftc.teamcode.hardware.AzusaImpl
import org.firstinspires.ftc.teamcode.hardware.BaseAzusa
import org.firstinspires.ftc.teamcode.util.AzusaTelemetry
import org.firstinspires.ftc.teamcode.util.Pose

@TeleOp
class AzusaNewTeleOp : BaseOpMode() {
    private val azusa: AzusaImpl = AzusaImpl(Pose(), hardwareMap, AzusaTelemetry(this))

    override val getRobot: BaseAzusa
        get() = azusa

    override fun onLoop() {
        azusa.driveTrain.teleopControl(gamepad1, false, azusa.currPose.h)
    }

}