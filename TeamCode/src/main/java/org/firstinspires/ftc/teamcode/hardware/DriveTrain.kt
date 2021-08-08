package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.DataPacket
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
    private val motors: Array<ExpansionHubMotor> =
        arrayOf(frontLeft, frontRight, backLeft, backRight)

    override fun update(dp: DataPacket) {
        val rawFrontLeft: Double = powers.y + powers.x + powers.h.angle
        val rawFrontRight: Double = powers.y - powers.x - powers.h.angle
        val rawBackLeft: Double = powers.y - powers.x + powers.h.angle
        val rawBackRight: Double = powers.y + powers.x - powers.h.angle
        val rawPowers = doubleArrayOf(rawFrontLeft, rawFrontRight, rawBackLeft, rawBackRight)

        var maxAbsPower = rawFrontLeft.absoluteValue
        for (power in rawPowers)
            if (power.absoluteValue > maxAbsPower) maxAbsPower = power.absoluteValue

        if (maxAbsPower > 1)
            for (i in rawPowers.indices) rawPowers[i] /= maxAbsPower

        for (i in rawPowers.indices) motors[i].power = rawPowers[i]

        dp.addData("power vectors", powers)
        dp.addData("powers", rawPowers.contentToString())
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
