package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.util.opmode.AzusaTelemetry
import org.openftc.revextensions2.ExpansionHubMotor

class Intake(private val leftIntake: ExpansionHubMotor, private val rightIntake: ExpansionHubMotor) : Hardware() {
    private var leftPower = 0.0
    private var rightPower = 0.0

    override fun update(azuTelemetry: AzusaTelemetry) {
        leftIntake.power = leftPower
        rightIntake.power = rightPower
    }

    fun setPowers(left: Double, right: Double) {
        leftPower = left
        rightPower = right
    }

    init {
        leftIntake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightIntake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
}
