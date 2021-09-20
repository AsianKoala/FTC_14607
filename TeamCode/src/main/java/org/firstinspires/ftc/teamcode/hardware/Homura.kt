package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.util.hw.DTPowers
import org.firstinspires.ftc.teamcode.util.math.Point
import org.firstinspires.ftc.teamcode.util.opmode.AzusaTelemetry
import org.openftc.revextensions2.ExpansionHubMotor
import kotlin.math.absoluteValue
import kotlin.math.max

class Homura(leftFirst: ExpansionHubMotor,
             leftSecond: ExpansionHubMotor,
             rightFirst: ExpansionHubMotor,
             rightSecond: ExpansionHubMotor): Hardware() {
    private val motors = mutableListOf(leftFirst, leftSecond, rightFirst, rightSecond)

    var powers: DTPowers = DTPowers()
    var trackwidth: Double = 14.0

    override fun update(azuTelemetry: AzusaTelemetry) {
        var left = powers.fwd - powers.trn
        var right = -powers.fwd - powers.trn
        val mx = max(left.absoluteValue, right.absoluteValue)
        if(mx > 1.0) {
            left /= mx
            right /= mx
        }
        motors[0].power = left
        motors[1].power = left
        motors[2].power = right
        motors[3].power = right
    }

    override fun disable() {
        powers = DTPowers(Point.ORIGIN)
    }

    init {
        for(m in motors) {
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            m.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }
}