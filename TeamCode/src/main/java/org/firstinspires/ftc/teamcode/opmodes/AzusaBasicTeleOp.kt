package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.DriveTrain
import org.firstinspires.ftc.teamcode.util.DataPacket
import org.firstinspires.ftc.teamcode.util.Pose
import org.openftc.revextensions2.ExpansionHubMotor

@TeleOp
@Disabled
class AzusaBasicTeleOp : OpMode() {

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
        val driveScale = 0.65 - if (gamepad1.left_bumper) 0.3 else 0.0
        driveTrain.powers = Pose(
            -gamepad1.left_stick_x * driveScale,
            gamepad1.left_stick_y * driveScale,
            -gamepad1.right_stick_x * driveScale
        )
        driveTrain.update(DataPacket())
    }
}
