package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.util.opmode.AzusaTelemetry

abstract class Hardware() {
    abstract fun update(azuTelemetry: AzusaTelemetry)
    abstract fun disable()
}
