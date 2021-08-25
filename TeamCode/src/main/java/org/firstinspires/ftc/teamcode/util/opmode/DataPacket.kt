package org.firstinspires.ftc.teamcode.util.opmode

import com.acmerobotics.dashboard.telemetry.TelemetryPacket

class DataPacket : TelemetryPacket() {
    fun addData(key: String, `val`: Any) {
        addLine("$key: $`val`")
    }

    fun addSpace() {
        addLine(" ")
    }
}
