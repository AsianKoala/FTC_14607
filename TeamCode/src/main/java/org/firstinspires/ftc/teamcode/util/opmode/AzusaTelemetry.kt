package org.firstinspires.ftc.teamcode.util.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl

class AzusaTelemetry(opMode: OpMode) {
    private var dataPacket: DataPacket = DataPacket()
    private val telemetry: TelemetryImpl = TelemetryImpl(opMode)

    fun addData(key: String, `val`: Any) {
        telemetry.addData(key, `val`)
        dataPacket.addData(key, `val`)
    }

    fun addSpace() {
        telemetry.addLine()
        dataPacket.addLine(" ")
    }

    fun fieldOverlay(): Canvas {
        return dataPacket.fieldOverlay()
    }

    fun update() {
        telemetry.update()
        FtcDashboard.getInstance().sendTelemetryPacket(dataPacket)
        dataPacket = DataPacket()
    }
}
