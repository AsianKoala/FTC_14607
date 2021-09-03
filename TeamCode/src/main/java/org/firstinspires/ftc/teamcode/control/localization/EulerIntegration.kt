package org.firstinspires.ftc.teamcode.control.localization

import org.firstinspires.ftc.teamcode.util.math.Point
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin

object EulerIntegration {
    fun update(dx: Double, lDelta: Double, rDelta: Double, dh: Double): Point {
        return if (dh.absoluteValue > 0) {
            val radiusOfMovement = (lDelta + rDelta) / (2 * dh)
            val radiusOfStrafe = dx / dh

            Point(
                radiusOfMovement * (1 - cos(dh)) + (radiusOfStrafe * sin(dh)),
                (radiusOfMovement * sin(dh)) + (radiusOfStrafe * (1 - cos(dh)))
            )
        } else Point(dx, (lDelta - rDelta) / 2.0)
    }
}
