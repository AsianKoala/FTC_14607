package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.openftc.revextensions2.ExpansionHubMotor
import kotlin.math.* // ktlint-disable no-wildcard-imports

@TeleOp
class TestKt : OpMode() {

    lateinit var frontLeft: ExpansionHubMotor
    lateinit var frontRight: ExpansionHubMotor
    lateinit var backLeft: ExpansionHubMotor
    lateinit var backRight: ExpansionHubMotor
    lateinit var driveTrain: DriveTrain

    override fun init() {
        frontLeft = hardwareMap.get(ExpansionHubMotor::class.java, "FL")
        frontRight = hardwareMap.get(ExpansionHubMotor::class.java, "FR")
        backLeft = hardwareMap.get(ExpansionHubMotor::class.java, "BL")
        backRight = hardwareMap.get(ExpansionHubMotor::class.java, "BR")
        driveTrain = DriveTrain(frontLeft, frontRight, backLeft, backRight)
    }

    override fun loop() {
        telemetry.addLine(driveTrain.powers.toString())
    }
}

class DriveTrain(
    frontLeft: ExpansionHubMotor,
    frontRight: ExpansionHubMotor,
    backLeft: ExpansionHubMotor,
    backRight: ExpansionHubMotor
) {

    var powers: Pose
    private val motors: Array<ExpansionHubMotor> =
        arrayOf(frontLeft, frontRight, backLeft, backRight)

    fun update() {
        val rawFrontLeft: Double = powers.y + powers.x + powers.h.raw
        val rawFrontRight: Double = powers.y - powers.x - powers.h.raw
        val rawBackLeft: Double = powers.y - powers.x + powers.h.raw
        val rawBackRight: Double = powers.y + powers.x - powers.h.raw
        val rawPowers = doubleArrayOf(rawFrontLeft, rawFrontRight, rawBackLeft, rawBackRight)

        var maxAbsPower = rawFrontLeft.absoluteValue
        for (power in rawPowers)
            if (power.absoluteValue > maxAbsPower) maxAbsPower = power.absoluteValue

        if (maxAbsPower > 1)
            for (i in rawPowers.indices) rawPowers[i] /= maxAbsPower

        for (i in rawPowers.indices)
            motors[i].power = rawPowers[i]
    }

    init {
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = DcMotorSimple.Direction.REVERSE
        for (m in motors) {
            m.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        powers = Pose(Point(), Angle(Angle.Unit.RAW))
    }
}

data class Point(
    var x: Double = 0.0,
    var y: Double = 0.0
) {
    val hypot = hypot(x, y)
    val atan2 = Angle(atan2(y, x))
    val dbNormalize get() = Point(y, x)
    val copy get() = Point(x, y)

    operator fun plus(p: Point) = Point(x + p.x, y + p.y)
    operator fun minus(p: Point) = Point(x - p.x, y + p.y)
    operator fun times(n: Double) = Point(x * n, y * n)
    operator fun div(n: Double) = Point(x / n, y / n)
    operator fun unaryMinus() = this.times(-1.0)

    fun distance(p: Point) = minus(p).hypot
    fun rotate(angle: Double) = Point(
        x * cos(angle) - y * sin(angle),
        x * sin(angle) + y * cos(angle)
    )

    override fun toString() = String.format("(%.2f, %.2f", x, y)
}

data class Pose(
    var p: Point = Point(),
    var h: Angle = Angle()
) {
    constructor(x: Double, y: Double, h: Double) : this(
        Point(
            x,
            y
        ),
        Angle(h)
    )
    constructor(p: Point, h: Double = 0.0) : this(p.x, p.y, h)
    constructor(p: Pose2d) : this(p.x, p.y, p.heading)

    val x = p.x
    val y = p.y
    val cos = h.cos
    val sin = h.sin
    val copy get() = Pose(p, h)
}

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

    operator fun times(scalar: Double) = Angle(angle * scalar, unit)
    operator fun div(scalar: Double) = Angle(angle / scalar, unit)
}
