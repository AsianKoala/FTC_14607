package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.control.path.Path
import org.firstinspires.ftc.teamcode.control.system.AzusaDeprecated
import org.firstinspires.ftc.teamcode.util.Angle
import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose

@TeleOp
@Disabled
@Deprecated("deprecated with azusa")
class AzusaTeleOp : AzusaDeprecated() {
    override fun startPose(): Pose {
        return Pose(Point.ORIGIN, Angle(0.0, Angle.Unit.RAD))
    }

    override fun path(): Path? {
        return null
    }

    override fun loop() {
        super.loop()
        controlGamepad()
    }

    private fun controlGamepad() {
        if (pathCache == null) {
            val driveScale: Double = 0.65 - if (gamepad1.left_bumper) 0.3 else 0.0
            driveTrain.powers = Pose(Point(
                -gamepad1.left_stick_x * driveScale,
                gamepad1.left_stick_y * driveScale),
                Angle(-gamepad1.right_stick_x * driveScale, Angle.Unit.RAD)
            )
        }
    }
}
