package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.control.localization.Speedometer
import org.firstinspires.ftc.teamcode.control.localization.ThreeWheelOdometry
import org.firstinspires.ftc.teamcode.util.AzusaTelemetry
import org.firstinspires.ftc.teamcode.util.Pose
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor
import org.openftc.revextensions2.RevBulkData
import kotlin.collections.ArrayList

@Config
class AzusaImpl(val startPose: Pose, val hwMap: HardwareMap, val telemetry: AzusaTelemetry) : BaseAzusa() {

    override lateinit var allHardware: ArrayList<Hardware>

    lateinit var masterHub: ExpansionHubEx
    lateinit var masterBulkData: RevBulkData

    lateinit var odometry: ThreeWheelOdometry
    lateinit var driveTrain: DriveTrain
    lateinit var speedometer: Speedometer

    lateinit var currPose: Pose
    lateinit var currVel: Pose

    override fun init() {
        allHardware = ArrayList()

        masterHub = hwMap.get(ExpansionHubEx::class.java, "masterHub")
        masterBulkData = masterHub.bulkInputData

        odometry = ThreeWheelOdometry(startPose)
        speedometer = Speedometer()

        driveTrain = DriveTrain(
            hwMap.get(ExpansionHubMotor::class.java, "FL"),
            hwMap.get(ExpansionHubMotor::class.java, "FR"),
            hwMap.get(ExpansionHubMotor::class.java, "BL"),
            hwMap.get(ExpansionHubMotor::class.java, "BR")
        )

        allHardware.add(driveTrain)
    }

    override fun update() {
        masterBulkData = masterHub.bulkInputData

        currPose = odometry.update(
            masterBulkData.getMotorCurrentPosition(0),
            masterBulkData.getMotorCurrentPosition(1),
            masterBulkData.getMotorCurrentPosition(2)
        )

        currVel = speedometer.update(currPose.h)


        allHardware.forEach { it.update() }
    }
}
