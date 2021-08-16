package org.firstinspires.ftc.teamcode.util

data class TimePose(
    @JvmField var p: Pose = Pose(Point.ORIGIN, Angle(Angle.Unit.RAD)),
    @JvmField var time: Long = System.currentTimeMillis()
)
