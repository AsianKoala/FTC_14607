package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.DriveTrain

@TeleOp
@Disabled
class BasicTeleOp : OpMode() {
    override fun init() {
        telemetry.update()
    }

    private lateinit var driveTrain: DriveTrain

    override fun loop() {
        controlGamePad()
    }

    private fun controlGamePad() {
        val x = -gamepad1.left_stick_x * 1.0
        val y = gamepad1.left_stick_y * 1.0
        val h = -gamepad1.right_stick_x * 1.0

//        driveTrain.powers = Pose(x, y, h)
    }
}
