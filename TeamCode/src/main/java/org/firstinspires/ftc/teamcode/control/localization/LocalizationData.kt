package org.firstinspires.ftc.teamcode.control.localization

import org.firstinspires.ftc.teamcode.util.Pose

data class LocalizationData(val finalPose: Pose, val deltaXVec: Double, val deltaYVec: Double)
