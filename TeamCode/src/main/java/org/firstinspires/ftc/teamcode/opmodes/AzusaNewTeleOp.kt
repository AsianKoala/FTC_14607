package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.builders.PathBuilder
import org.firstinspires.ftc.teamcode.control.path.waypoints.StopWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import org.firstinspires.ftc.teamcode.control.system.BaseOpMode
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.Point
import org.firstinspires.ftc.teamcode.util.math.Pose
import kotlin.math.PI

@TeleOp
class AzusaNewTeleOp : BaseOpMode() {

    override fun startPose(): Pose = Pose(Point(38.0, 58.0), Angle(PI, AngleUnit.RAD))

    lateinit var pathCache: Path

    lateinit var x: DoubleArray
    lateinit var y: DoubleArray

    override fun onInit() {
        pathCache = PathBuilder().addPoint(Waypoint(38.0, 58.0, 0.0))
            .addPoint(Waypoint(15.0, 54.0, 6.0))
            .addPoint(Waypoint(2.0, 46.0, 6.0))
            .addPoint(Waypoint(-8.0, 32.0, 6.0))
            .addPoint(Waypoint(-12.0, 10.0, 6.0))
            .addPoint(Waypoint(-14.0, -8.0, 6.0))
            .addPoint(Waypoint(-8.0, -14.0, 10.0))
            .addPoint(Waypoint(0.0, -14.0, 6.0))
            .addPoint(Waypoint(12.0, -6.0, 6.0))
            .addPoint(Waypoint(16.0, 6.0, 6.0))
            .addPoint(Waypoint(12.0, 16.0, 6.0))
            .addPoint(Waypoint(0.0, 28.0, 10.0))
            .addPoint(Waypoint(6.0, 42.0, 6.0))
            .addPoint(StopWaypoint(28.0, 54.0, 6.0, Angle(0.0, AngleUnit.RAD))).build()

        x = DoubleArray(pathCache.waypoints.size)
        y = DoubleArray(pathCache.waypoints.size)
        for ((index, e) in pathCache.waypoints.withIndex()) {
            x[index] = e.p.y
            y[index] = -e.p.x
        }
    }

    override fun onInitLoop() {
        azusaTelemetry.fieldOverlay().strokePolyline(x, y)
    }

    override fun onLoop() {
        if (gamepad1.right_trigger > 0.4 && !pathCache.finished()) {
            pathCache.follow(azusa)
        } else {
            azusa.teleopControl(gamepad1, 0.5)
        }
        azusaTelemetry.fieldOverlay().strokePolyline(x, y)
    }
}
