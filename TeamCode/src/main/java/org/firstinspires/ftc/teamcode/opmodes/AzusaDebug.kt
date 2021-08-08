package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.path.PathPoint
import org.firstinspires.ftc.teamcode.control.path.StopPathPoint
import org.firstinspires.ftc.teamcode.control.path.builders.PathBuilder
import org.firstinspires.ftc.teamcode.control.system.Azusa
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.Debuggable
import org.firstinspires.ftc.teamcode.util.Pose

@Debuggable
@TeleOp
class AzusaDebug : Azusa() {
    override fun startPose() = Pose(-64.0, -64.0, Math.PI / 2)

    override fun path(): Path {
        val exp = PathBuilder("lCurve exp")
            .addPoint(PathPoint("start", -64.0, -64.0, 0.0))
            .addPoint(PathPoint("fw", -64.0, -34.0, 14.0))
            .addPoint(PathPoint("joint 1", -46.0, -10.0, 14.0))
            .addPoint(PathPoint("joint 2", -16.0, 8.0, 14.0))
            .addPoint(PathPoint("track 1", 16.0, 12.0, 14.0))
            .addPoint(StopPathPoint("track 2", 56.0, 12.0, 14.0, Angle(0.0)))
        return exp.build()
    }
}
