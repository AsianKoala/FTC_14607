package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.builders.PathBuilder
import org.firstinspires.ftc.teamcode.control.path.waypoints.PointTurnWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.StopWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import org.firstinspires.ftc.teamcode.control.system.BaseAuto
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.MathUtil.toRadians
import org.firstinspires.ftc.teamcode.util.math.Point
import org.firstinspires.ftc.teamcode.util.math.Pose
import kotlin.math.PI

@Autonomous
class AzusaExtAuto : BaseAuto() {
    override fun startPose(): Pose = Pose(Point(88.0, 88.0), Angle(PI, AngleUnit.RAD))

    override fun initialPath(): Path {

        return PathBuilder(0.55)
            .addPoint(Waypoint(88.0, 88.0, 0.0))
            .addPoint(Waypoint(65.0, 84.0, 6.0))
            .addPoint(Waypoint(45.0, 80.0, 6.0))
            .addPoint(Waypoint(32.0, 76.0, 6.0))
            .addPoint(Waypoint(26.0, 72.0, 6.0))
            .addPoint(Waypoint(20.0, 68.0, 6.0))
            .addPoint(Waypoint(16.0, 62.0, 6.0))
            .addPoint(Waypoint(12.0, 40.0, 6.0))
            .addPoint(Waypoint(10.0, 20.0, 6.0))
            .addPoint(Waypoint(16.0, 10.0, 6.0))
            .addPoint(Waypoint(30.0, 12.0, 6.0))
            .addPoint(Waypoint(42.0, 24.0, 6.0))
            .addPoint(Waypoint(46.0, 36.0, 6.0))
            .addPoint(Waypoint(42.0, 46.0, 6.0))
            .addPoint(Waypoint(30.0, 58.0, 10.0))
            .addPoint(Waypoint(36.0, 72.0, 12.0))
            .addPoint(Waypoint(45.0, 76.0, 10.0))
            .addPoint(StopWaypoint(84.0, 84.0, 6.0, Angle(0.2, AngleUnit.RAD)))
            .addPoint(PointTurnWaypoint(84.0, 84.0, 0.0, Angle(0.0, AngleUnit.RAD), Angle(2.0.toRadians, AngleUnit.RAD))).build()
    }
}
