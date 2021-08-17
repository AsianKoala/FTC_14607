package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.util.MathUtil.toDegrees
import kotlin.math.* // ktlint-disable no-wildcard-imports

data class Angle(
    var angle: Double,
    var unit: AngleUnit
) {

    private val fullCircle get() = when (unit) {
        AngleUnit.RAD -> PI * 2
        AngleUnit.RAW -> 0.0
    }

    private val halfCircle get() = when (unit) {
        AngleUnit.RAD -> PI
        AngleUnit.RAW -> 0.0
    }

    val deg: Double
        get() = when (unit) {
            AngleUnit.RAD -> angle.toDegrees
            AngleUnit.RAW -> angle
        }

    val rad: Double
        get() = when (unit) {
            AngleUnit.RAD -> angle
            AngleUnit.RAW -> angle
        }

    val raw: Double
        get() = when (unit) {
            AngleUnit.RAD -> angle
            AngleUnit.RAW -> angle
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
        AngleUnit.RAD -> Angle(rad + other.rad, unit).wrap()
        AngleUnit.RAW -> Angle(raw + other.raw, unit)
    }

    operator fun minus(other: Angle) = plus(other.unaryMinus())

    operator fun unaryMinus() = when (unit) {
        AngleUnit.RAD -> Angle(-rad, unit).wrap()
        AngleUnit.RAW -> Angle(-raw, unit)
    }

    operator fun times(scalar: Double) = Angle(angle * scalar, unit)
    operator fun div(scalar: Double) = Angle(angle / scalar, unit)
}
