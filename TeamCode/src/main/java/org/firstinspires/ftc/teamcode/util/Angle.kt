package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.util.MathUtil.toDegrees
import org.firstinspires.ftc.teamcode.util.MathUtil.toRadians
import kotlin.math.* // ktlint-disable no-wildcard-imports

data class Angle(
    var angle: Double,
    var unit: Unit
) {
    enum class Unit {
        RAD,
        RAW
    }

    private val fullCircle get() = when (unit) {
        Unit.RAD -> PI * 2
        Unit.RAW -> 0.0
    }

    private val halfCircle get() = when (unit) {
        Unit.RAD -> PI
        Unit.RAW -> 0.0
    }

    val deg: Double
        get() = when (unit) {
            Unit.RAD -> angle.toDegrees
            Unit.RAW -> angle
        }

    val rad: Double
        get() = when (unit) {
            Unit.RAD -> angle
            Unit.RAW -> angle
        }

    val raw: Double
        get() = when (unit) {
            Unit.RAD -> angle
            Unit.RAW -> angle
        }

    val cos = cos(angle)
    val sin = sin(angle)
    val sign = angle.sign
    val abs = angle.absoluteValue
    val copy get() = Angle(angle, unit)

    fun wrap(): Angle {
        var heading = angle
        while (heading < -halfCircle)
            heading += fullCircle
        while (heading > halfCircle)
            heading -= fullCircle
        return Angle(heading, unit)
    }

    operator fun plus(other: Angle) = when (unit) {
        Unit.RAD -> Angle(rad + other.rad, unit).wrap()
        Unit.RAW -> Angle(raw + other.raw, unit)
    }

    operator fun minus(other: Angle) = plus(other.unaryMinus())

    operator fun unaryMinus() = when (unit) {
        Unit.RAD -> Angle(-rad, unit).wrap()
        Unit.RAW -> Angle(-raw, unit)
    }

    operator fun times(scalar: Double) = Angle(angle * scalar, unit)
    operator fun div(scalar: Double) = Angle(angle / scalar, unit)
}
