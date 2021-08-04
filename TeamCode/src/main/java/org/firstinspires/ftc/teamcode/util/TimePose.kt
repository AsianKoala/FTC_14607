package org.firstinspires.ftc.teamcode.util

data class TimePose(
    @JvmField var p: Pose = Pose(),
    @JvmField var time: Long = System.currentTimeMillis()
)