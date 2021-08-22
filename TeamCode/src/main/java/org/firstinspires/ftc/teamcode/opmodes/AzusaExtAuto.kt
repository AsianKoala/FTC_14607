package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.builders.PathBuilder
import org.firstinspires.ftc.teamcode.control.path.waypoints.LockedWaypoint
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
    override fun startPose(): Pose = Pose(Point(38.0, 58.0), Angle(PI, AngleUnit.RAD))

    override fun initialPath(): Path {
        return PathBuilder(1.0).addPoint(Waypoint(38.0, 58.0, 0.0))
            .addPoint(Waypoint(15.0, 54.0, 13.0))
            .addPoint(Waypoint(-8.0, 32.0, 13.0))
            .addPoint(Waypoint(-12.0, 10.0, 13.0))
            .addPoint(LockedWaypoint(-14.0, -8.0, 13.0, Angle(255.0.toRadians, AngleUnit.RAD)))
            .addPoint(LockedWaypoint(0.0, -10.0, 13.0, Angle(255.0.toRadians, AngleUnit.RAD)))
            .addPoint(Waypoint(12.0, -2.0, 13.0))
            .addPoint(Waypoint(16.0, 6.0, 13.0))
            .addPoint(Waypoint(12.0, 16.0, 13.0))
            .addPoint(Waypoint(0.0, 28.0, 13.0))
            .addPoint(Waypoint(6.0, 42.0, 13.0))
            .addPoint(StopWaypoint(28.0, 54.0, 13.0, Angle(0.0, AngleUnit.RAD))).build()
    }
}
