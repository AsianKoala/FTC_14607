package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.system.BaseOpMode
import org.firstinspires.ftc.teamcode.util.*

@Debuggable
@TeleOp
@Disabled
class AzusaNewDebug : BaseOpMode() {
    override fun startPose(): Pose = Pose(Point.ORIGIN, Angle(0.0, AngleUnit.RAD))

    override fun onLoop() {
        azusa.debugControl(gamepad1)
    }
}
