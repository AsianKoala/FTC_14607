package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.Azusa
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose

@TeleOp
class CringeTeleOp: OpMode() {

    lateinit var azusa: Azusa
    override fun init() {
        azusa = Azusa(Pose(Point.ORIGIN, Angle(Angle.Unit.RAD)), false)
        azusa.init(hardwareMap, telemetry)
    }

    override fun loop() {
        azusa.update()
        azusa.teleopControl(gamepad1)
    }
}