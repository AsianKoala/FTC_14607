package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.control.controllers.PurePursuitController
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.StopWaypoint
import org.firstinspires.ftc.teamcode.control.path.Waypoint
import org.firstinspires.ftc.teamcode.control.path.builders.PathBuilder
import org.firstinspires.ftc.teamcode.control.system.BaseOpMode
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.AngleUnit
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose
import kotlin.math.PI

@Autonomous
class AzusaNewAuto : BaseOpMode() {
    override fun startPose(): Pose = Pose(Point(0.0, 0.0), Angle(PI / 4, AngleUnit.RAD))

    fun getPath(): Path {
        return PathBuilder()
            .addPoint(Waypoint("diag", 24.0, 24.0, 8.0))
            .addPoint(Waypoint()).build()
    }

    override fun onLoop() {
        PurePursuitController.goToPosition(azusa, StopWaypoint("final", 24.0, 24.0, 8.0, Angle(PI + PI / 4, AngleUnit.RAD)), 0.3)

        if (azusa.currPose.p.distance(Point(24.0, 24.0)) < 0.5) {
            azusa.driveTrain.setZeroPowers()
            azusaTelemetry.addData("FINAL POSITION REACHED", "")
        }
    }
}
