package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.util.MathUtil.toDegrees
import org.firstinspires.ftc.teamcode.util.MathUtil.toRadians
import kotlin.math.* // ktlint-disable no-wildcard-imports

data class Angle(
    @JvmField var angle: Double = 0.0,
    @JvmField var unit: Unit = Unit.RAD
) {
    constructor(unit: Unit) : this(0.0, unit)

    enum class Unit {
        RAD,
        DEG,
        RAW
    }

    private val FULL_CIRCLE = when (unit) {
        Unit.RAD -> PI * 2
        Unit.DEG -> 360.0
        Unit.RAW -> 0.0
    }

    private val HALF_CIRCLE = when (unit) {
        Unit.RAD -> PI
        Unit.DEG -> 180.0
        Unit.RAW -> 0.0
    }

    val deg: Double
        get() = when (unit) {
            Unit.DEG -> angle
            Unit.RAD -> angle.toDegrees
            Unit.RAW -> angle
        }

    val rad: Double
        get() = when (unit) {
            Unit.DEG -> angle.toRadians
            Unit.RAD -> angle
            Unit.RAW -> angle
        }

    val raw: Double
        get() = when (unit) {
            Unit.DEG -> angle
            Unit.RAD -> angle
            Unit.RAW -> angle
        }

    val cos = cos(angle)
    val sin = sin(angle)
    val tan = tan(angle)
    val sign = angle.sign
    val abs = angle.absoluteValue

    companion object {
        fun createUnwrappedDeg(deg: Double) = Angle(deg, Unit.DEG)
        fun createUnwrappedRad(rad: Double) = Angle(rad, Unit.RAD)
        fun createWrappedDeg(deg: Double) = createUnwrappedDeg(deg).wrap()
        fun createWrappedRad(rad: Double) = createUnwrappedRad(rad).wrap()
    }

    fun wrap(): Angle {
        var heading = angle
        while (heading < -HALF_CIRCLE)
            heading += FULL_CIRCLE
        while (heading > HALF_CIRCLE)
            heading -= FULL_CIRCLE
        return Angle(heading, unit)
    }

    operator fun plus(other: Angle) = when (unit) {
        Unit.RAD -> createUnwrappedRad(rad + other.rad)
        Unit.DEG -> createUnwrappedDeg(deg + other.deg)
        Unit.RAW -> Angle(angle + other.angle, unit)
    }

    operator fun minus(other: Angle) = plus(other.unaryMinus())

    operator fun unaryMinus() = when (unit) {
        Unit.RAD -> createUnwrappedRad(-rad)
        Unit.DEG -> createUnwrappedDeg(-deg)
        else -> Angle(-angle, unit)
    }

    operator fun times(scalar: Double) = Angle(angle * scalar, unit)
    operator fun div(scalar: Double) = Angle(angle / scalar, unit)
}
