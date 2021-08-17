package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.AzusaTelemetry

abstract class Hardware {
    abstract fun update(azuTelemetry: AzusaTelemetry) // LinkedHashMap<String, String>
}
