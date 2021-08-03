package org.firstinspires.ftc.teamcode.util

import android.annotation.SuppressLint
import com.acmerobotics.roadrunner.geometry.Pose2d

open class Pose @JvmOverloads constructor(x: Double = 0.0, y: Double = 0.0, @JvmField var h: Double = 0.0) :
    Point(x, y) {
    constructor(p: Pose) : this(p.x, p.y, p.h) {}
    constructor(pose2d: Pose2d) : this(pose2d.x, pose2d.y, pose2d.heading) {}
    constructor(p: Point?, h: Double) : this(p!!.x, p.y, h) {}

    // im going to trust myself to angle wrap if it comes to it
    fun add(p: Pose): Pose {
        return Pose(super.add(p), h + p.h)
    }

    operator fun minus(p: Pose): Pose {
        return this.add(Pose(-p.x, -p.y, -p.h))
    }

    fun multiply(p: Pose): Pose {
        return divide(Pose(1 / p.x, 1 / p.y, 1 / p.h))
    }

    override fun scale(a: Double): Pose? {
        return multiply(Pose(a, a, a))
    }

    fun divide(p: Pose): Pose {
        return Pose(super.divide(p), h / p.h)
    }

    override fun abs(): Pose {
        return Pose(super.abs(), Math.abs(h))
    }

    override fun sgns(): Pose {
        return Pose(
            MathUtil.sgn(x).toDouble(), MathUtil.sgn(y)
                .toDouble(), MathUtil.sgn(
                h
            ).toDouble()
        )
    }

    fun pow(p: Pose): Pose {
        return Pose(super.pow(this), Math.pow(h, p.h))
    }

    fun cos(): Double {
        return Math.cos(h)
    }

    fun sin(): Double {
        return Math.sin(h)
    }

    fun relVals(target: Point): Pose {
        val d = minus(target).hypot()
        val r_h = target.minus(this).atan() - h
        val r_x = -d * Math.sin(r_h)
        val r_y = d * Math.cos(r_h)
        return Pose(r_x, r_y, r_h) // return 0 just to kmake sure neer to use it kek
    }

    fun minify(mins: Pose): Pose {
        val n = Pose(this)
        if (MathUtil.absGreater(x, y)) {
            if (MathUtil.absGreater(x, h)) {
                n.x = MathUtil.minVal(x, mins.x)
            } else {
                n.h = MathUtil.minVal(h, mins.h)
            }
        } else {
            if (MathUtil.absGreater(y, h)) {
                n.y = MathUtil.minVal(y, mins.y)
            } else {
                n.h = MathUtil.minVal(h, mins.h)
            }
        }
        return n
    }

    override fun clipAbs(max: Double): Pose? {
        val ret = Pose(this)
        if (MathUtil.absGreater(x, max)) {
            ret.x = sgns().x * max
        }
        if (MathUtil.absGreater(y, max)) {
            ret.y = sgns().y * max
        }
        if (MathUtil.absGreater(h, max)) {
            ret.h = sgns().h * max
        }
        return ret
    }

    fun wrap(): Pose {
        return Pose(x, y, MathUtil.angleWrap(h))
    }

    @SuppressLint("DefaultLocale")
    override fun toString(): String {
        return String.format("(%.1f, %.1f, %.1f)", x, y, h)
    }
}