package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.StopWaypoint
import org.firstinspires.ftc.teamcode.control.path.Waypoint
import org.firstinspires.ftc.teamcode.control.path.builders.PathBuilder
import org.firstinspires.ftc.teamcode.control.system.AzusaDeprecated
import org.firstinspires.ftc.teamcode.util.*

@Debuggable
@TeleOp
@Deprecated("deprecated with azusa")
@Disabled
class AzusaDebug : AzusaDeprecated() {
    override fun startPose() = Pose(Point(-64.0, -64.0), Angle(Math.PI / 2, AngleUnit.RAD))

    override fun path(): Path {
        val exp = PathBuilder()
            .addPoint(Waypoint("start", -64.0, -64.0, 0.0))
            .addPoint(Waypoint("fw", -64.0, -34.0, 14.0))
            .addPoint(Waypoint("joint 1", -46.0, -10.0, 14.0))
            .addPoint(Waypoint("joint 2", -16.0, 8.0, 14.0))
            .addPoint(Waypoint("track 1", 16.0, 12.0, 14.0))
            .addPoint(StopWaypoint("track 2", 56.0, 12.0, 14.0, Angle(0.0, AngleUnit.RAW)))
        return exp.build()
    }
}
