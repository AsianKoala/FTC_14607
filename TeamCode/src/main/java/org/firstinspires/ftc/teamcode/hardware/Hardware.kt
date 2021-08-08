package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.util.DataPacket

abstract class Hardware {
    abstract fun update(dp: DataPacket)
}
