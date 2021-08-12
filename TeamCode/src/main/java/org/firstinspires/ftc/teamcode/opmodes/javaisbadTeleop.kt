package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.openftc.revextensions2.ExpansionHubMotor
import java.util.ArrayList

@TeleOp
class javaisbadTeleop : OpMode() {
    private lateinit var driveTrain: DriveTrain
    override fun init() {
        val frontLeft = hardwareMap.get(ExpansionHubMotor::class.java, "FL")
        val frontRight = hardwareMap.get(ExpansionHubMotor::class.java, "FR")
        val backLeft = hardwareMap.get(ExpansionHubMotor::class.java, "BL")
        val backRight = hardwareMap.get(ExpansionHubMotor::class.java, "BR")
        driveTrain = DriveTrain(frontLeft, frontRight, backLeft, backRight)
    }

    override fun loop() {
        val scale: Double = if (gamepad1.left_bumper) 1.0 else 0.5
        driveTrain.powers = Pose(
            -gamepad1.left_stick_x * scale,
            gamepad1.left_stick_y * scale,
            -gamepad1.right_stick_x * scale
        )
        driveTrain.update()
        telemetry.addData("left", driveTrain.frontLeft.currentPosition)
        telemetry.addData("right", driveTrain.frontRight.currentPosition)
        telemetry.addData("perp", driveTrain.backLeft.currentPosition)
    }
}

internal class Pose(var x: Double, var y: Double, var h: Double)
internal class DriveTrain(
    var frontLeft: ExpansionHubMotor,
    var frontRight: ExpansionHubMotor,
    var backLeft: ExpansionHubMotor,
    backRight: ExpansionHubMotor
) {
    var powers: Pose
    private val motors = ArrayList<ExpansionHubMotor>()
    fun update() {
        val rawFL = powers.y + powers.x + powers.h
        val rawFR = powers.y - powers.x - powers.h
        val rawBL = powers.y - powers.x + powers.h
        val rawBR = powers.y + powers.x - powers.h
        val rawPowers = doubleArrayOf(rawFL, rawFR, rawBL, rawBR)
        var maxabs = rawPowers[0]
        for (i in rawPowers) {
            if (Math.abs(i) > maxabs) {
                maxabs = Math.abs(i)
            }
        }
        if (maxabs > 1) {
            for (i in rawPowers.indices) {
                rawPowers[i] /= maxabs
            }
        }
        for (i in rawPowers.indices) {
            motors[i].power = rawPowers[i]
        }
    }

    init {
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = DcMotorSimple.Direction.REVERSE
        motors.add(frontLeft)
        motors.add(frontRight)
        motors.add(backLeft)
        motors.add(backRight)
        for (m in motors) {
            m.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        powers = Pose(0.0, 0.0, 0.0)
    }
}
