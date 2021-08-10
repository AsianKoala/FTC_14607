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

    private val fullCircle = when (unit) {
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
    val sign = angle.sign
    val abs = angle.absoluteValue

    fun wrap(): Angle {
        var heading = angle
        while (heading < -HALF_CIRCLE)
            heading += fullCircle
        while (heading > HALF_CIRCLE)
            heading -= fullCircle
        return Angle(heading, unit)
    }

    operator fun plus(other: Angle) = when (unit) {
        Unit.RAD -> Angle(rad + other.rad, unit).wrap()
        Unit.DEG -> Angle(deg + other.deg, unit).wrap()
        Unit.RAW -> Angle(raw + other.raw, unit)
    }

    operator fun minus(other: Angle) = plus(other.unaryMinus())

    operator fun unaryMinus() = when (unit) {
        Unit.RAD -> Angle(-rad, unit).wrap()
        Unit.DEG -> Angle(-deg, unit).wrap()
        Unit.RAW -> Angle(-raw, unit)
    }

    operator fun times(scalar: Double) = Angle(angle * scalar, unit)
    operator fun div(scalar: Double) = Angle(angle / scalar, unit)
}
