package org.firstinspires.ftc.teamcode.control.localization

import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.MathUtil
import org.firstinspires.ftc.teamcode.util.MathUtil.epsilonEquals
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose
import kotlin.math.cos
import kotlin.math.sin

object ArcLocalizer {
    fun update(currentPosition: Pose, baseDeltas: Pose, finalAngle: Angle): LocalizationData {
        val dtheta = baseDeltas.h.angle
        val (sineTerm, cosTerm) = if (dtheta epsilonEquals 0.0) {
            1.0 - dtheta * dtheta / 6.0 to dtheta / 2.0
        } else {
            sin(dtheta) to (1.0 - cos(dtheta)) / dtheta
        }
        val dx = sineTerm * baseDeltas.p.y - cosTerm * baseDeltas.p.x
        val dy = cosTerm * baseDeltas.p.y + sineTerm * baseDeltas.p.x

        val deltas = Point(dx, dy)

        val finalDelta = MathUtil.rotatePoint(deltas, currentPosition.h)
        val finalPose = Pose(currentPosition.p + finalDelta, finalAngle)

        Speedometer.deltas += baseDeltas.p
        return LocalizationData(finalPose, deltas.x, deltas.y)
    }
}
