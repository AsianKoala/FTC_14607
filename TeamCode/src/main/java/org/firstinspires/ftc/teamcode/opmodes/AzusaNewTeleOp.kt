package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.builders.PathBuilder
import org.firstinspires.ftc.teamcode.control.system.BaseOpMode
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.AngleUnit
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose
import kotlin.math.PI

@TeleOp
class AzusaNewTeleOp : BaseOpMode() {

    override fun startPose(): Pose = Pose(Point(16.0, -18.0), Angle(PI / 2, AngleUnit.RAD))

    lateinit var pathCache: Path

    lateinit var x: DoubleArray
    lateinit var y: DoubleArray

    override fun onInit() {
        pathCache = PathBuilder().build()
//        pathCache = PathBuilder()
//            .addPoint(Waypoint("start", startPose().x, startPose().y, 0.0))
//            .addPoint(Waypoint("forward", 18.0, 14.0, 4.0))
//            .addPoint(Waypoint("first bend", 21.0, 30.0, 4.0))
//            .addPoint(Waypoint("second bend", 29.0, 36.0, 4.0))
//            .addPoint(Waypoint("third bend", 42.0, 33.0, 4.0))
//            .addPoint(Waypoint("fourth bend", 44.0, 22.0, 4.0))
//            .addPoint(Waypoint("exit", 40.0, 12.0, 4.0))
//            .addPoint(Waypoint("exit real", 30.0, 0.0, 4.0))
//            .addPoint(StopWaypoint("stop", 18.0, -13.0, 6.0, Angle(4.167, AngleUnit.RAD))).build()

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
            pathCache.follow(azusa, 0.5)
        } else {
            azusa.teleopControl(gamepad1, 0.5)
        }
        azusaTelemetry.fieldOverlay().strokePolyline(x, y)
    }
}
