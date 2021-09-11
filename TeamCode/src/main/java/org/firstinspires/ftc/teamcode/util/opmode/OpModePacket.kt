package org.firstinspires.ftc.teamcode.util.opmode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.math.Pose

data class OpModePacket(val startPose: Pose, val debugging: Boolean, val hwMap: HardwareMap,
                        val telemetry: AzusaTelemetry, val gamepad: Gamepad, val gamepad2: Gamepad)
