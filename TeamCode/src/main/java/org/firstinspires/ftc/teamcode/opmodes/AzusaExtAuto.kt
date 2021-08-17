package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.PathPoint
import org.firstinspires.ftc.teamcode.control.path.StopPathPoint
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
        return PathBuilder("main path")
                .addPoint(PathPoint("first", 16.0, 16.0, 8.0))
                .addPoint(PathPoint("waterbottle upbend", 20.0, 28.0, 8.0))
                .addPoint(PathPoint("peak", 28.0, 36.0, 8.0))
                .addPoint(PathPoint("waterbottle downbend", 34.0, 28.0, 8.0))
                .addPoint(PathPoint("waterbottle downbend 2", 32.0, 18.0, 8.0))
                .addPoint(PathPoint("smoother 1", 25.0, 10.0, 8.0))
                .addPoint(StopPathPoint("smoother 2", 18.0, 5.0, 8.0, Angle(PI, AngleUnit.RAD))).build()
    }

    override fun startPose(): Pose = Pose(Point.ORIGIN, Angle(PI / 2, AngleUnit.RAD))
}