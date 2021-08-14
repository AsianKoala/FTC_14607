package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.robotcore.external.Telemetry

abstract class Hardware {
    abstract fun update(telemetry: Telemetry) // LinkedHashMap<String, String>
}
