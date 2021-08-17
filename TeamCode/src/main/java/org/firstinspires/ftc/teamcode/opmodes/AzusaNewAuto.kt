package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.control.controllers.PurePursuitController
import org.firstinspires.ftc.teamcode.control.path.StopPathPoint
import org.firstinspires.ftc.teamcode.control.system.BaseOpMode
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.MathUtil.toRadians
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose
import kotlin.math.PI

@Autonomous
class AzusaNewAuto : BaseOpMode() {
    override fun startPose(): Pose = Pose(Point(0.0, 0.0), Angle(PI/2, Angle.Unit.RAD))

    override fun onLoop() {
        PurePursuitController.gunToPosition(azusa, StopPathPoint("final", 25.0, 25.0, 8.0, Angle(45.0.toRadians, Angle.Unit.RAD)), 0.3)

        if (azusa.currPose.p.distance(Point(25.0, 25.0)) < 1.0) {
            azusa.driveTrain.setZeroPowers()
            azusaTelemetry.addData("FINAL POSITION REACHED", "")
        }
    }
}
