package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.system.BaseOpMode
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.Point
import org.firstinspires.ftc.teamcode.util.math.Pose

@TeleOp
class AzusaTele : BaseOpMode() {

    override fun startPose(): Pose = Pose(Point.ORIGIN, Angle(0.0, AngleUnit.RAD))

    override fun onLoop() {
        azusa.teleopControl(0.5, false)
    }
}
