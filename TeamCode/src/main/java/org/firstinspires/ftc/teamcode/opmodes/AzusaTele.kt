package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.builders.PurePursuitBuilder
import org.firstinspires.ftc.teamcode.control.path.waypoints.LockedWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.StopWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import org.firstinspires.ftc.teamcode.control.system.BaseOpMode
import org.firstinspires.ftc.teamcode.util.debug.Debuggable
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.MathUtil.toRadians
import org.firstinspires.ftc.teamcode.util.math.Point
import org.firstinspires.ftc.teamcode.util.math.Pose
import org.firstinspires.ftc.teamcode.util.opmode.Globals
import kotlin.math.PI

@TeleOp
class AzusaTele : BaseOpMode() {

    override val startPose = if(Globals.IS_COMP) Globals.AUTO_END_POSE.copy else Pose(Point(38.0, 58.0), Angle.EAST)

    lateinit var pathCache: Path
    override fun onInit() {
        pathCache = PurePursuitBuilder().addPoint(Waypoint(38.0, 58.0, 0.0))
                .addPoint(Waypoint(15.0, 54.0, 14.0))
                .addPoint(Waypoint(0.0, 46.0, 14.0))
                .addPoint(LockedWaypoint(-8.0, 32.0, 14.0, Angle(270.0.toRadians, AngleUnit.RAD)))
                .addPoint(LockedWaypoint(-8.0, 10.0, 14.0, Angle(270.0.toRadians, AngleUnit.RAD)))
                .addPoint(LockedWaypoint(-8.0, -8.0, 14.0, Angle(270.0.toRadians, AngleUnit.RAD)))
                .addPoint(Waypoint(0.0, -10.0, 14.0))
                .addPoint(Waypoint(12.0, -2.0, 12.0))
                .addPoint(Waypoint(16.0, 6.0, 12.0))
                .addPoint(Waypoint(12.0, 16.0, 12.0))
                .addPoint(Waypoint(0.0, 28.0, 10.0))
                .addPoint(Waypoint(6.0, 42.0, 10.0))
                .addPoint(StopWaypoint(28.0, 54.0, 10.0, Angle(PI, AngleUnit.RAD), Angle(3.0.toRadians, AngleUnit.RAD))).build()
    }

    override fun onLoop() {
        azusa.teleopControl(0.5, true)
    }
}
