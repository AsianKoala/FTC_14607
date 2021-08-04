package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.system.Azusa
import org.firstinspires.ftc.teamcode.util.Pose

@TeleOp
class AzusaTeleOp : Azusa() {
    override fun startPose(): Pose {
        return Pose(0.0, 0.0, 0.0)
    }

    override fun path(): Path? {
        return null
    }

    override fun loop() {
        super.loop()
        controlGamepad()
    }

    private fun controlGamepad() {
        if (pathCache.isEmpty()) {
            val driveScale: Double = 0.65 - if (gamepad1.left_bumper) 0.3 else 0.0
            driveTrain.powers = Pose(
                -gamepad1.left_stick_x * driveScale,
                gamepad1.left_stick_y * driveScale,
                -gamepad1.right_stick_x * driveScale
            )
        }
    }
}