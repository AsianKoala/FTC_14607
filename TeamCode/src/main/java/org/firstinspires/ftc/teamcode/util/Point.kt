package org.firstinspires.ftc.teamcode.util

import android.annotation.SuppressLint
import kotlin.math.*;

open class Point constructor(@JvmField var x: Double = 0.0, @JvmField var y: Double = 0.0) {
    constructor(p: Point) : this(p.x, p.y)
    constructor(a: Double) : this(a, a)

    fun rotated(angle: Double): Point {
        val newX = x * cos(angle) - y * sin(angle)
        val newY = x * sin(angle) + y * cos(angle)
        return Point(newX, newY)
    }

    fun add(p: Point): Point {
        return Point(x + p.x, y + p.y)
    }

    operator fun minus(p: Point): Point {
        return add(Point(-p.x, -p.y))
    }

    fun divide(p: Point): Point {
        return Point(Point(x / p.x, y / p.y))
    }

    fun multiply(p: Point): Point {
        return divide(Point(1 / p.x, 1 / p.y))
    }

    open fun scale(a: Double): Point? {
        return multiply(Point(a, a))
    }

    fun pow(p: Point): Point {
        return Point(x.pow(p.x), y.pow(p.y))
    }

    open fun abs(): Point {
        return Point(abs(x), abs(y))
    }

    open fun sgns(): Point {
        return Point(
            MathUtil.sgn(
                x
            ).toDouble(), MathUtil.sgn(y).toDouble()
        )
    }

//    val hypot get()  = hypot(x, y)

    fun hypot(): Double {
        return sqrt(x* x + y * y)
    }

    fun distance(p: Point): Double {
        return hypot(x - p.x, y - p.y)
    }

    fun atan(): Double {
        return atan2(y, x)
    }

    fun dbNormalize(): Point {
        return Point(-(-y), -x)
    }

    open fun clipAbs(max: Double): Point? {
        val ret = Point(this)
        if (MathUtil.absGreater(x, max)) {
            ret.x = sgns().x * max
        }
        if (MathUtil.absGreater(y, max)) {
            ret.y = sgns().y * max
        }
        return ret
    }

    @SuppressLint("DefaultLocale")
    override fun toString(): String {
        return String.format("(%.1f, %.1f)", x, y)
    }
}