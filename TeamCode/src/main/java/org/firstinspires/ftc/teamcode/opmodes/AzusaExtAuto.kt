package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.StopWaypoint
import org.firstinspires.ftc.teamcode.control.path.Waypoint
import org.firstinspires.ftc.teamcode.control.path.builders.PathBuilder
import org.firstinspires.ftc.teamcode.control.system.BaseAuto
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.AngleUnit
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose
import kotlin.math.PI

@Autonomous
class AzusaExtAuto : BaseAuto() {
    override fun initialPath(): Path {
        return PathBuilder()
            .addPoint(Waypoint("start", startPose().x, startPose().y, 0.0))
            .addPoint(Waypoint("forward", 18.0, 14.0, 4.0))
            .addPoint(Waypoint("first bend", 21.0, 30.0, 4.0))
            .addPoint(Waypoint("second bend", 29.0, 36.0, 4.0))
            .addPoint(Waypoint("third bend", 38.0, 33.0, 4.0))
            .addPoint(Waypoint("fourth bend", 40.0, 28.0, 4.0))
            .addPoint(Waypoint("exit", 36.0, 15.0, 4.0))
            .addPoint(StopWaypoint("stop", startPose().x, startPose().y, 6.0, Angle(4.167, AngleUnit.RAD))).build()
    }

    override fun startPose(): Pose = Pose(Point.ORIGIN, Angle(PI / 2, AngleUnit.RAD))
}
