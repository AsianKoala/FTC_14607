package org.firstinspires.ftc.teamcode.util

data class TimePose(
        @JvmField var p: Pose = Pose(Point.ORIGIN, Angle(0.0, AngleUnit.RAD)),
        @JvmField var time: Long = System.currentTimeMillis()
)
