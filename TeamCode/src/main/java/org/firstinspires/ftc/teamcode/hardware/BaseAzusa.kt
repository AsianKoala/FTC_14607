package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.util.AzusaTelemetry

abstract class BaseAzusa {
    abstract val telemetry: AzusaTelemetry
    abstract fun init()
    abstract fun updateData()
    abstract fun update()
}