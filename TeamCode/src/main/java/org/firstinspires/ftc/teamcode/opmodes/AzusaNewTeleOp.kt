package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.controllers.PurePursuitController
import org.firstinspires.ftc.teamcode.control.path.StopPathPoint
import org.firstinspires.ftc.teamcode.control.system.BaseOpMode
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.AngleUnit
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose
import kotlin.math.PI

@TeleOp
class AzusaNewTeleOp : BaseOpMode() {

    override fun startPose(): Pose = Pose(Point.ORIGIN, Angle(PI / 2, AngleUnit.RAD))

    override fun onLoop() {
        if (gamepad1.right_trigger > 0.4) {
            PurePursuitController.goToPosition(azusa, StopPathPoint("stop", 24.0, 24.0, 8.0, Angle(PI + PI / 4, AngleUnit.RAD)), 0.3)
        } else {
            azusa.teleopControl(gamepad1, 0.3)
        }
    }
}
// 54-1964039
