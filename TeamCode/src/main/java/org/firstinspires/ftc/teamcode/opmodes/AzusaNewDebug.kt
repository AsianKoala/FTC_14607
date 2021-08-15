package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.system.BaseOpMode
import org.firstinspires.ftc.teamcode.util.Debuggable
import org.firstinspires.ftc.teamcode.util.Pose

@Debuggable
@TeleOp
class AzusaNewDebug : BaseOpMode() {
    override fun startPose(): Pose = Pose()

    override fun onLoop() {
        azusa.debugControl(gamepad1)
    }
}