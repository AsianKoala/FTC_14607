package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.Point
import org.firstinspires.ftc.teamcode.util.math.Pose
import org.firstinspires.ftc.teamcode.util.opmode.OpModePacket
import org.openftc.revextensions2.ExpansionHubMotor

class Firefly(dataPacket: OpModePacket) : BaseRobot(dataPacket) {

    lateinit var driveTrain: DriveTrain
    lateinit var intake: Intake

    override fun init() {
        driveTrain = DriveTrain(
            dataPacket.hwMap[ExpansionHubMotor::class.java, "FL"],
            dataPacket.hwMap[ExpansionHubMotor::class.java, "FR"],
            dataPacket.hwMap[ExpansionHubMotor::class.java, "BL"],
            dataPacket.hwMap[ExpansionHubMotor::class.java, "BR"]
        )

        intake = Intake(
            dataPacket.hwMap[ExpansionHubMotor::class.java, "leftIntake"],
            dataPacket.hwMap[ExpansionHubMotor::class.java, "rightIntake"]
        )

        allHardware.add(driveTrain)
        allHardware.add(intake)
    }

    override fun update() {
        allHardware.forEach { it.update(dataPacket.telemetry) }
    }

    fun teleopControl(driveScale: Double) {
        driveTrain.powers = Pose(
            Point(
                dataPacket.gamepad.left_stick_x * driveScale,
                -dataPacket.gamepad.left_stick_y * driveScale
            ),
            Angle(-dataPacket.gamepad.right_stick_x * driveScale, AngleUnit.RAW)
        )

        intake.setPowers(dataPacket.gamepad.left_bumper.b2d, dataPacket.gamepad.right_bumper.b2d)
    }

    private val Boolean.b2d get() = if (this) 1.0 else 0.0
}
