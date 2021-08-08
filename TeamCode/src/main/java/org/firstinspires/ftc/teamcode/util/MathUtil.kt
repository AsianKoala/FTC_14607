package org.firstinspires.ftc.teamcode.util

import java.util.*
import kotlin.math.*

object MathUtil {
    const val EPSILON = 1e-6
    const val TAU = 2 * PI

    val Double.toRadians get() = Math.toRadians(this)
    val Double.toDegrees get() = Math.toDegrees(this)

    fun epsilonEquals(a: Double, b: Double): Boolean {
        return when (a.isInfinite()) {
            true -> a == b
            false -> (a - b).absoluteValue < EPSILON
        }
    }

    fun angleThresh(a: Angle, b: Angle, c: Angle): Boolean {
        return (a - b).wrap().rad.absoluteValue < c.rad
    }

    fun maxify(input: Double, min: Double): Double {
        return when (input) {
            in 0.0..min -> min
            in -min..0.0 -> -min
            else -> input
        }
    }

    fun clipIntersection(start: Point, end: Point, robot: Point): Point {
        if (start.y == end.y)
            start.y += 0.01
        if (start.x == end.x)
            start.x += 0.01

        val m1 = (end.y - start.y) / (end.x - start.x)
        val m2 = -1.0 / m1
        val xClip = (-m2 * robot.x + robot.y + m1 * start.x - start.y) / (m1 - m2)
        val yClip = m1 * (xClip - start.x) + start.y
        return Point(xClip, yClip)
    }

    fun extendLine(firstPoint: Point, secondPoint: Point, distance: Double): Point {
        val lineAngle = atan2(secondPoint.y - firstPoint.y, secondPoint.x - firstPoint.x)
        val lineLength = hypot(secondPoint.y - firstPoint.y, secondPoint.x - firstPoint.x)
        val extendedLineLength = lineLength + distance
        val extended = secondPoint.copy
        extended.x = cos(lineAngle) * extendedLineLength + firstPoint.x
        extended.y = sin(lineAngle) * extendedLineLength + firstPoint.y
        return extended
    }

    // change s_tart to a_start
    // change e_nd to a_end
    // change r_obot to c_x and c_y
    /**
     * Returns the closest intersection point to the end of a line segment created through the intersection of a line and circle.
     * The main purpose of this is for pure pursuit but I'll prob implement it into our goToPosition algorithm.
     * For pure pursuit use, c would be the clipped robot point, startPoint would be the current segment start point,
     * endPoint would be the current segment end point, and radius would be our follow distance
     *
     * @param center          center point of circle
     * @param startPoint start point of the line segment
     * @param endPoint   end point of the line segment
     * @param radius     radius of the circle
     * @return intersection point closest to endPoint
     * @see [https://mathworld.wolfram.com/Circle-LineIntersection.html](https://mathworld.wolfram.com/Circle-LineIntersection.html)
     */
    fun circleLineIntersection(
        center: Point,
        startPoint: Point,
        endPoint: Point,
        radius: Double
    ): Point {
        val start = startPoint - center
        val end = endPoint - center
        val deltas = end - start
        val d = start.x * end.y - end.x * start.y
        val discriminant = radius.pow(2) * deltas.hypot.pow(2) - d.pow(2)

        // discriminant = 0 for 1 intersection, >0 for 2
        val intersections = ArrayList<Point>()
        val xLeft = d * deltas.y
        val yLeft = -d * deltas.x
        val xRight: Double = deltas.y.sign * deltas.x * sqrt(discriminant)
        val yRight = deltas.y.absoluteValue * sqrt(discriminant)
        val div = deltas.hypot.pow(2)
        if (discriminant == 0.0) {
            intersections.add(Point(xLeft / div, yLeft / div))
        } else {
            // add 2 points, one with positive right side and one with negative right side
            intersections.add(Point((xLeft + xRight) / div, (yLeft + yRight) / div))
            intersections.add(Point((xLeft - xRight) / div, (yLeft - yRight) / div))
        }
        var closest = Point(-10000.0, -10000.0)
        for (p in intersections) { // add circle center offsets
            p.x += center.x
            p.y += center.y
            if (p.distance(endPoint) < closest.distance(endPoint)) closest = p
        }
        return closest
    }
}
