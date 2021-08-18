package org.firstinspires.ftc.teamcode.control.localization.old

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.MathUtil
import org.firstinspires.ftc.teamcode.util.math.MathUtil.epsilonEquals
import org.firstinspires.ftc.teamcode.util.math.Point
import org.firstinspires.ftc.teamcode.util.math.Pose
import kotlin.math.cos
import kotlin.math.sin

object ArcLocalizer {
    fun update(currentPosition: Pose, baseDeltas: Pose, finalAngle: Angle, telemetry: Telemetry): LocalizationData {
        val dtheta = baseDeltas.h.angle
        telemetry.addData("dtheta", dtheta)
        val (sineTerm, cosTerm) = if (dtheta epsilonEquals 0.0) {
            1.0 - dtheta * dtheta / 6.0 to dtheta / 2.0
        } else {
            sin(dtheta) / dtheta to (1.0 - cos(dtheta)) / dtheta
        }
        val dx = sineTerm * baseDeltas.p.x - cosTerm * baseDeltas.p.y
        val dy = cosTerm * baseDeltas.p.x + sineTerm * baseDeltas.p.y

        telemetry.addData("dx", dx)
        telemetry.addData("dy", dy)

        val deltas = Point(dx, dy)

        val finalDelta = MathUtil.rotatePoint(deltas, currentPosition.h)
        val finalPose = Pose(currentPosition.p + finalDelta, finalAngle)

        telemetry.addData("rot delta x", finalDelta.x)
        telemetry.addData("rot delta y", finalDelta.y)

//        Speedometer.deltas += baseDeltas.p
        return LocalizationData(finalPose, deltas.x, deltas.y)
    }
}
