package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.control.localization.ThreeWheelOdometry
import org.firstinspires.ftc.teamcode.control.system.BaseOpMode
import org.firstinspires.ftc.teamcode.util.AllianceSide
import org.firstinspires.ftc.teamcode.util.AzusaTelemetry
import org.firstinspires.ftc.teamcode.util.Pose
import org.openftc.revextensions2.ExpansionHubEx

@Config
class AzusaImpl(val startPose: Pose, val hwMap: HardwareMap, val telemetry: AzusaTelemetry) : BaseAzusa() {
    lateinit var masterHub: ExpansionHubEx
    lateinit var odometry: ThreeWheelOdometry
    lateinit var driveTrain: DriveTrain

    override fun init() {
        TODO("Not yet implemented")
    }

    override fun update() {
        TODO("Not yet implemented")
    }
}

class testAuto() : BaseOpMode() {
    val azusa: AzusaImpl = AzusaImpl(Pose(), hardwareMap, AzusaTelemetry(this))

    override val getAzusaImpl: BaseAzusa get() = azusa
    override val getAllianceImpl: AllianceSide get() = AllianceSide.BLUE

    override fun onLoop() {
    }
}
