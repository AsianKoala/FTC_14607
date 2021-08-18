package org.firstinspires.ftc.teamcode.util.math

data class TimePose(
    @JvmField var p: Pose = Pose(Point.ORIGIN, Angle(0.0, AngleUnit.RAD)),
    @JvmField var time: Long = System.currentTimeMillis()
)
