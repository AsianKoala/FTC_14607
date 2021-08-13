package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose
import org.openftc.revextensions2.ExpansionHubMotor
import kotlin.math.absoluteValue

class DriveTrain(
    frontLeft: ExpansionHubMotor,
    frontRight: ExpansionHubMotor,
    backLeft: ExpansionHubMotor,
    backRight: ExpansionHubMotor
) : Hardware() {

    var powers: Pose
    private val motors = arrayOf(frontLeft, frontRight, backLeft, backRight)

    override fun update(): LinkedHashMap<String, String> {
        val rawFrontLeft: Double = powers.y + powers.x + powers.h.raw
        val rawFrontRight: Double = powers.y - powers.x - powers.h.raw
        val rawBackLeft: Double = powers.y - powers.x + powers.h.raw
        val rawBackRight: Double = powers.y + powers.x - powers.h.raw

        var rawPowers = listOf(rawFrontLeft, rawFrontRight, rawBackLeft, rawBackRight)
        val max: Double = rawPowers.map { it.absoluteValue }.maxOrNull()!!
        if (max > 1.0) rawPowers = rawPowers.map { it / max }

        motors.forEachIndexed { i, it -> it.power = rawPowers[i] }

        val dp = LinkedHashMap<String, String>()
        dp["power vectors"] = powers.toRawString
        dp["powers"] = rawPowers.toString()
        return dp
    }

    init {
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = DcMotorSimple.Direction.REVERSE
        for (m in motors) {
            m.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        powers = Pose(Point(), Angle(0.0, Angle.Unit.RAW))
    }
}
