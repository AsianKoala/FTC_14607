package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.controllers.PurePursuitController
import org.firstinspires.ftc.teamcode.control.path.PathPoint
import org.firstinspires.ftc.teamcode.control.system.BaseOpMode
import org.firstinspires.ftc.teamcode.util.Pose

@TeleOp
class AzusaNewTeleOp : BaseOpMode() {

    private var currFollowPoint: PathPoint? = null
    override fun startPose(): Pose = Pose()

    override fun onLoop() {
        if (currFollowPoint != null) {
            PurePursuitController.goToPosition(azusa, currFollowPoint!!)
        } else {
            azusa.teleopControl(gamepad1)
        }
    }
}